// pbrt is Copyright(c) 1998-2020 Matt Pharr, Wenzel Jakob, and Greg Humphreys.
// The pbrt source code is licensed under the Apache License, Version 2.0.
// SPDX: Apache-2.0

#include <pbrt/cameras.h>

#include <pbrt/base/light.h>
#include <pbrt/base/medium.h>
#include <pbrt/bsdf.h>
#include <pbrt/film.h>
#include <pbrt/lights.h>
#include <pbrt/options.h>
#include <pbrt/paramdict.h>
#include <pbrt/util/error.h>
#include <pbrt/util/file.h>
#include <pbrt/util/image.h>
#include <pbrt/util/lowdiscrepancy.h>
#include <pbrt/util/math.h>
#include <pbrt/util/parallel.h>
#include <pbrt/util/print.h>
#include <pbrt/util/sampling.h>
#include <pbrt/util/stats.h>
#include <pbrt/util/progressreporter.h>

#include <algorithm>
#include <cmath>

namespace pbrt {

// CameraTransform Method Definitions
CameraTransform::CameraTransform(const AnimatedTransform& worldFromCamera) {
    switch (Options->renderingSpace) {
    case RenderingCoordinateSystem::Camera: {
        // Compute _worldFromRender_ for camera-space rendering
        Float tMid = (worldFromCamera.startTime + worldFromCamera.endTime) / 2;
        worldFromRender = worldFromCamera.Interpolate(tMid);
        break;
    }
    case RenderingCoordinateSystem::CameraWorld: {
        // Compute _worldFromRender_ for camera-world space rendering
        Float tMid = (worldFromCamera.startTime + worldFromCamera.endTime) / 2;
        Point3f pCamera = worldFromCamera(Point3f(0, 0, 0), tMid);
        worldFromRender = Translate(Vector3f(pCamera));
        break;
    }
    case RenderingCoordinateSystem::World: {
        // Compute _worldFromRender_ for world-space rendering
        worldFromRender = Transform();
        break;
    }
    default:
        LOG_FATAL("Unhandled rendering coordinate space");
    }
    LOG_VERBOSE("World-space position: %s", worldFromRender(Point3f(0, 0, 0)));
    // Compute _renderFromCamera_ transformation
    Transform renderFromWorld = Inverse(worldFromRender);
    Transform rfc[2] = {renderFromWorld * worldFromCamera.startTransform,
                        renderFromWorld * worldFromCamera.endTransform};
    renderFromCamera = AnimatedTransform(rfc[0], worldFromCamera.startTime, rfc[1],
                                         worldFromCamera.endTime);
}

std::string CameraTransform::ToString() const {
    return StringPrintf("[ CameraTransform renderFromCamera: %s worldFromRender: %s ]",
                        renderFromCamera, worldFromRender);
}

// Camera Method Definitions
PBRT_CPU_GPU pstd::optional<CameraRayDifferential> Camera::GenerateRayDifferential(
    CameraSample sample, SampledWavelengths& lambda) const {
    auto gen = [&](auto ptr) { return ptr->GenerateRayDifferential(sample, lambda); };
    return Dispatch(gen);
}

PBRT_CPU_GPU SampledSpectrum Camera::We(const Ray& ray, SampledWavelengths& lambda,
                                        Point2f* pRaster2) const {
    auto we = [&](auto ptr) { return ptr->We(ray, lambda, pRaster2); };
    return Dispatch(we);
}

PBRT_CPU_GPU void Camera::PDF_We(const Ray& ray, Float* pdfPos, Float* pdfDir) const {
    auto pdf = [&](auto ptr) { return ptr->PDF_We(ray, pdfPos, pdfDir); };
    return Dispatch(pdf);
}

PBRT_CPU_GPU pstd::optional<CameraWiSample> Camera::SampleWi(
    const Interaction& ref, Point2f u,
    SampledWavelengths& lambda) const {
    auto sample = [&](auto ptr) { return ptr->SampleWi(ref, u, lambda); };
    return Dispatch(sample);
}

void Camera::InitMetadata(ImageMetadata* metadata) const {
    auto init = [&](auto ptr) { return ptr->InitMetadata(metadata); };
    return DispatchCPU(init);
}

std::string Camera::ToString() const {
    if (!ptr())
        return "(nullptr)";

    auto ts = [&](auto ptr) { return ptr->ToString(); };
    return DispatchCPU(ts);
}

// CameraBase Method Definitions
CameraBase::CameraBase(CameraBaseParameters p)
    : cameraTransform(p.cameraTransform),
      shutterOpen(p.shutterOpen),
      shutterClose(p.shutterClose),
      film(p.film),
      medium(p.medium) {
    if (cameraTransform.CameraFromRenderHasScale())
        Warning("Scaling detected in rendering space to camera space transformation!\n"
            "The system has numerous assumptions, implicit and explicit,\n"
            "that this transform will have no scale factors in it.\n"
            "Proceed at your own risk; your image may have errors or\n"
            "the system may crash as a result of this.");
}

PBRT_CPU_GPU pstd::optional<CameraRayDifferential> CameraBase::GenerateRayDifferential(
    Camera camera, CameraSample sample, SampledWavelengths& lambda) {
    // Generate regular camera ray _cr_ for ray differential
    pstd::optional<CameraRay> cr = camera.GenerateRay(sample, lambda);
    if (!cr)
        return {};
    RayDifferential rd(cr->ray);

    // Find camera ray after shifting one pixel in the $x$ direction
    pstd::optional<CameraRay> rx;
    for (Float eps : {.05f, -.05f}) {
        CameraSample sshift = sample;
        sshift.pFilm.x += eps;
        // Try to generate ray with _sshift_ and compute $x$ differential
        if (rx = camera.GenerateRay(sshift, lambda); rx) {
            rd.rxOrigin = rd.o + (rx->ray.o - rd.o) / eps;
            rd.rxDirection = rd.d + (rx->ray.d - rd.d) / eps;
            break;
        }
    }

    // Find camera ray after shifting one pixel in the $y$ direction
    pstd::optional<CameraRay> ry;
    for (Float eps : {.05f, -.05f}) {
        CameraSample sshift = sample;
        sshift.pFilm.y += eps;
        if (ry = camera.GenerateRay(sshift, lambda); ry) {
            rd.ryOrigin = rd.o + (ry->ray.o - rd.o) / eps;
            rd.ryDirection = rd.d + (ry->ray.d - rd.d) / eps;
            break;
        }
    }

    // Return approximate ray differential and weight
    rd.hasDifferentials = rx && ry;
    return CameraRayDifferential{rd, cr->weight};
}

void CameraBase::FindMinimumDifferentials(Camera camera) {
    minPosDifferentialX = minPosDifferentialY = minDirDifferentialX =
                                                minDirDifferentialY = Vector3f(
                                                    Infinity, Infinity, Infinity);

    CameraSample sample;
    sample.pLens = Point2f(0.5, 0.5);
    sample.time = 0.5;
    SampledWavelengths lambda = SampledWavelengths::SampleVisible(0.5);

    int n = 512;
    for (int i = 0; i < n; ++i) {
        sample.pFilm.x = Float(i) / (n - 1) * film.FullResolution().x;
        sample.pFilm.y = Float(i) / (n - 1) * film.FullResolution().y;

        pstd::optional<CameraRayDifferential> crd =
            camera.GenerateRayDifferential(sample, lambda);
        if (!crd)
            continue;

        RayDifferential& ray = crd->ray;
        Vector3f dox = CameraFromRender(ray.rxOrigin - ray.o, ray.time);
        if (Length(dox) < Length(minPosDifferentialX))
            minPosDifferentialX = dox;
        Vector3f doy = CameraFromRender(ray.ryOrigin - ray.o, ray.time);
        if (Length(doy) < Length(minPosDifferentialY))
            minPosDifferentialY = doy;

        ray.d = Normalize(ray.d);
        ray.rxDirection = Normalize(ray.rxDirection);
        ray.ryDirection = Normalize(ray.ryDirection);

        Frame f = Frame::FromZ(ray.d);
        Vector3f df = f.ToLocal(ray.d); // should be (0, 0, 1);
        Vector3f dxf = Normalize(f.ToLocal(ray.rxDirection));
        Vector3f dyf = Normalize(f.ToLocal(ray.ryDirection));

        if (Length(dxf - df) < Length(minDirDifferentialX))
            minDirDifferentialX = dxf - df;
        if (Length(dyf - df) < Length(minDirDifferentialY))
            minDirDifferentialY = dyf - df;
    }

    LOG_VERBOSE("Camera min pos differentials: %s, %s", minPosDifferentialX,
                minPosDifferentialY);
    LOG_VERBOSE("Camera min dir differentials: %s, %s", minDirDifferentialX,
                minDirDifferentialY);
}

void CameraBase::InitMetadata(ImageMetadata* metadata) const {
    metadata->cameraFromWorld = cameraTransform.CameraFromWorld(shutterOpen).GetMatrix();
}

std::string CameraBase::ToString() const {
    return StringPrintf("cameraTransform: %s shutterOpen: %f shutterClose: %f film: %s "
                        "medium: %s minPosDifferentialX: %s minPosDifferentialY: %s "
                        "minDirDifferentialX: %s minDirDifferentialY: %s ",
                        cameraTransform, shutterOpen, shutterClose, film,
                        medium ? medium.ToString().c_str() : "(nullptr)",
                        minPosDifferentialX, minPosDifferentialY, minDirDifferentialX,
                        minDirDifferentialY);
}

std::string CameraSample::ToString() const {
    return StringPrintf("[ CameraSample pFilm: %s pLens: %s time: %f filterWeight: %f ]",
                        pFilm, pLens, time, filterWeight);
}

// ProjectiveCamera Method Definitions
void ProjectiveCamera::InitMetadata(ImageMetadata* metadata) const {
    metadata->cameraFromWorld = cameraTransform.CameraFromWorld(shutterOpen).GetMatrix();

    // TODO: double check this
    Transform NDCFromWorld = Translate(Vector3f(0.5, 0.5, 0.5)) * Scale(0.5, 0.5, 0.5) *
                             screenFromCamera * *metadata->cameraFromWorld;
    metadata->NDCFromWorld = NDCFromWorld.GetMatrix();

    CameraBase::InitMetadata(metadata);
}

std::string ProjectiveCamera::BaseToString() const {
    return CameraBase::ToString() +
           StringPrintf("screenFromCamera: %s cameraFromRaster: %s "
                        "rasterFromScreen: %s screenFromRaster: %s "
                        "lensRadius: %f focalDistance: %f",
                        screenFromCamera, cameraFromRaster, rasterFromScreen,
                        screenFromRaster, lensRadius, focalDistance);
}

Camera Camera::Create(const std::string& name, const ParameterDictionary& parameters,
                      Medium medium, const CameraTransform& cameraTransform, Film film,
                      const FileLoc* loc, Allocator alloc) {
    Camera camera;
    if (name == "perspective")
        camera = PerspectiveCamera::Create(parameters, cameraTransform, film, medium, loc,
                                           alloc);
    else if (name == "orthographic")
        camera = OrthographicCamera::Create(parameters, cameraTransform, film, medium,
                                            loc, alloc);
    else if (name == "realistic")
        camera = RealisticCamera::Create(parameters, cameraTransform, film, medium, loc,
                                         alloc);
    else if (name == "spherical")
        camera = SphericalCamera::Create(parameters, cameraTransform, film, medium, loc,
                                         alloc);
    else
        ErrorExit(loc, "%s: camera type unknown.", name);

    if (!camera)
        ErrorExit(loc, "%s: unable to create camera.", name);

    parameters.ReportUnused();
    return camera;
}

// CameraBaseParameters Method Definitions
CameraBaseParameters::CameraBaseParameters(const CameraTransform& cameraTransform,
                                           Film film, Medium medium,
                                           const ParameterDictionary& parameters,
                                           const FileLoc* loc)
    : cameraTransform(cameraTransform), film(film), medium(medium) {
    shutterOpen = parameters.GetOneFloat("shutteropen", 0.f);
    shutterClose = parameters.GetOneFloat("shutterclose", 1.f);
    if (shutterClose < shutterOpen) {
        Warning(loc, "Shutter close time %f < shutter open %f.  Swapping them.",
                shutterClose, shutterOpen);
        pstd::swap(shutterClose, shutterOpen);
    }
}

// OrthographicCamera Method Definitions
PBRT_CPU_GPU pstd::optional<CameraRay> OrthographicCamera::GenerateRay(
    CameraSample sample, SampledWavelengths& lambda) const {
    // Compute raster and camera sample positions
    Point3f pFilm = Point3f(sample.pFilm.x, sample.pFilm.y, 0);
    Point3f pCamera = cameraFromRaster(pFilm);

    Ray ray(pCamera, Vector3f(0, 0, 1), SampleTime(sample.time), medium);
    // Modify ray for depth of field
    if (lensRadius > 0) {
        // Sample point on lens
        Point2f pLens = lensRadius * SampleUniformDiskConcentric(sample.pLens);

        // Compute point on plane of focus
        Float ft = focalDistance / ray.d.z;
        Point3f pFocus = ray(ft);

        // Update ray for effect of lens
        ray.o = Point3f(pLens.x, pLens.y, 0);
        ray.d = Normalize(pFocus - ray.o);
    }

    return CameraRay{RenderFromCamera(ray)};
}

PBRT_CPU_GPU pstd::optional<CameraRayDifferential>
OrthographicCamera::GenerateRayDifferential(
    CameraSample sample, SampledWavelengths& lambda) const {
    // Compute main orthographic viewing ray
    // Compute raster and camera sample positions
    Point3f pFilm = Point3f(sample.pFilm.x, sample.pFilm.y, 0);
    Point3f pCamera = cameraFromRaster(pFilm);

    RayDifferential ray(pCamera, Vector3f(0, 0, 1), SampleTime(sample.time), medium);
    // Modify ray for depth of field
    if (lensRadius > 0) {
        // Sample point on lens
        Point2f pLens = lensRadius * SampleUniformDiskConcentric(sample.pLens);

        // Compute point on plane of focus
        Float ft = focalDistance / ray.d.z;
        Point3f pFocus = ray(ft);

        // Update ray for effect of lens
        ray.o = Point3f(pLens.x, pLens.y, 0);
        ray.d = Normalize(pFocus - ray.o);
    }

    // Compute ray differentials for _OrthographicCamera_
    if (lensRadius > 0) {
        // Compute _OrthographicCamera_ ray differentials accounting for lens
        // Sample point on lens
        Point2f pLens = lensRadius * SampleUniformDiskConcentric(sample.pLens);

        Float ft = focalDistance / ray.d.z;
        Point3f pFocus = pCamera + dxCamera + (ft * Vector3f(0, 0, 1));
        ray.rxOrigin = Point3f(pLens.x, pLens.y, 0);
        ray.rxDirection = Normalize(pFocus - ray.rxOrigin);

        pFocus = pCamera + dyCamera + (ft * Vector3f(0, 0, 1));
        ray.ryOrigin = Point3f(pLens.x, pLens.y, 0);
        ray.ryDirection = Normalize(pFocus - ray.ryOrigin);
    } else {
        ray.rxOrigin = ray.o + dxCamera;
        ray.ryOrigin = ray.o + dyCamera;
        ray.rxDirection = ray.ryDirection = ray.d;
    }

    ray.hasDifferentials = true;
    return CameraRayDifferential{RenderFromCamera(ray)};
}

std::string OrthographicCamera::ToString() const {
    return StringPrintf("[ OrthographicCamera %s dxCamera: %s dyCamera: %s ]",
                        BaseToString(), dxCamera, dyCamera);
}

OrthographicCamera* OrthographicCamera::Create(const ParameterDictionary& parameters,
                                               const CameraTransform& cameraTransform,
                                               Film film, Medium medium,
                                               const FileLoc* loc, Allocator alloc) {
    CameraBaseParameters cameraBaseParameters(cameraTransform, film, medium, parameters,
                                              loc);

    Float lensradius = parameters.GetOneFloat("lensradius", 0.f);
    Float focaldistance = parameters.GetOneFloat("focaldistance", 1e6f);
    Float frame =
        parameters.GetOneFloat("frameaspectratio", Float(film.FullResolution().x) /
                                                   Float(film.FullResolution().y));
    Bounds2f screen;
    if (frame > 1.f) {
        screen.pMin.x = -frame;
        screen.pMax.x = frame;
        screen.pMin.y = -1.f;
        screen.pMax.y = 1.f;
    } else {
        screen.pMin.x = -1.f;
        screen.pMax.x = 1.f;
        screen.pMin.y = -1.f / frame;
        screen.pMax.y = 1.f / frame;
    }
    std::vector<Float> sw = parameters.GetFloatArray("screenwindow");
    if (!sw.empty()) {
        if (Options->fullscreen) {
            Warning("\"screenwindow\" is ignored in fullscreen mode");
        } else {
            if (sw.size() == 4) {
                screen.pMin.x = sw[0];
                screen.pMax.x = sw[1];
                screen.pMin.y = sw[2];
                screen.pMax.y = sw[3];
            } else {
                Error("\"screenwindow\" should have four values");
            }
        }
    }
    return alloc.new_object<OrthographicCamera>(cameraBaseParameters, screen, lensradius,
                                                focaldistance);
}

// PerspectiveCamera Method Definitions
PBRT_CPU_GPU pstd::optional<CameraRay> PerspectiveCamera::GenerateRay(
    CameraSample sample, SampledWavelengths& lambda) const {
    // Compute raster and camera sample positions
    Point3f pFilm = Point3f(sample.pFilm.x, sample.pFilm.y, 0);
    Point3f pCamera = cameraFromRaster(pFilm);

    Ray ray(Point3f(0, 0, 0), Normalize(Vector3f(pCamera)), SampleTime(sample.time),
            medium);
    // Modify ray for depth of field
    if (lensRadius > 0) {
        // Sample point on lens
        Point2f pLens = lensRadius * SampleUniformDiskConcentric(sample.pLens);

        // Compute point on plane of focus
        Float ft = focalDistance / ray.d.z;
        Point3f pFocus = ray(ft);

        // Update ray for effect of lens
        ray.o = Point3f(pLens.x, pLens.y, 0);
        ray.d = Normalize(pFocus - ray.o);
    }

    return CameraRay{RenderFromCamera(ray)};
}

PBRT_CPU_GPU pstd::optional<CameraRayDifferential>
PerspectiveCamera::GenerateRayDifferential(
    CameraSample sample, SampledWavelengths& lambda) const {
    // Compute raster and camera sample positions
    Point3f pFilm = Point3f(sample.pFilm.x, sample.pFilm.y, 0);
    Point3f pCamera = cameraFromRaster(pFilm);
    Vector3f dir = Normalize(Vector3f(pCamera.x, pCamera.y, pCamera.z));
    RayDifferential ray(Point3f(0, 0, 0), dir, SampleTime(sample.time), medium);
    // Modify ray for depth of field
    if (lensRadius > 0) {
        // Sample point on lens
        Point2f pLens = lensRadius * SampleUniformDiskConcentric(sample.pLens);

        // Compute point on plane of focus
        Float ft = focalDistance / ray.d.z;
        Point3f pFocus = ray(ft);

        // Update ray for effect of lens
        ray.o = Point3f(pLens.x, pLens.y, 0);
        ray.d = Normalize(pFocus - ray.o);
    }

    // Compute offset rays for _PerspectiveCamera_ ray differentials
    if (lensRadius > 0) {
        // Compute _PerspectiveCamera_ ray differentials accounting for lens
        // Sample point on lens
        Point2f pLens = lensRadius * SampleUniformDiskConcentric(sample.pLens);

        // Compute $x$ ray differential for _PerspectiveCamera_ with lens
        Vector3f dx = Normalize(Vector3f(pCamera + dxCamera));
        Float ft = focalDistance / dx.z;
        Point3f pFocus = Point3f(0, 0, 0) + (ft * dx);
        ray.rxOrigin = Point3f(pLens.x, pLens.y, 0);
        ray.rxDirection = Normalize(pFocus - ray.rxOrigin);

        // Compute $y$ ray differential for _PerspectiveCamera_ with lens
        Vector3f dy = Normalize(Vector3f(pCamera + dyCamera));
        ft = focalDistance / dy.z;
        pFocus = Point3f(0, 0, 0) + (ft * dy);
        ray.ryOrigin = Point3f(pLens.x, pLens.y, 0);
        ray.ryDirection = Normalize(pFocus - ray.ryOrigin);
    } else {
        ray.rxOrigin = ray.ryOrigin = ray.o;
        ray.rxDirection = Normalize(Vector3f(pCamera) + dxCamera);
        ray.ryDirection = Normalize(Vector3f(pCamera) + dyCamera);
    }

    ray.hasDifferentials = true;
    return CameraRayDifferential{RenderFromCamera(ray)};
}

std::string PerspectiveCamera::ToString() const {
    return StringPrintf("[ PerspectiveCamera %s dxCamera: %s dyCamera: %s A: "
                        "%f cosTotalWidth: %f ]",
                        BaseToString(), dxCamera, dyCamera, A, cosTotalWidth);
}

PerspectiveCamera* PerspectiveCamera::Create(const ParameterDictionary& parameters,
                                             const CameraTransform& cameraTransform,
                                             Film film, Medium medium, const FileLoc* loc,
                                             Allocator alloc) {
    CameraBaseParameters cameraBaseParameters(cameraTransform, film, medium, parameters,
                                              loc);

    Float lensradius = parameters.GetOneFloat("lensradius", 0.f);
    Float focaldistance = parameters.GetOneFloat("focaldistance", 1e6);
    Float frame =
        parameters.GetOneFloat("frameaspectratio", Float(film.FullResolution().x) /
                                                   Float(film.FullResolution().y));
    Bounds2f screen;
    if (frame > 1.f) {
        screen.pMin.x = -frame;
        screen.pMax.x = frame;
        screen.pMin.y = -1.f;
        screen.pMax.y = 1.f;
    } else {
        screen.pMin.x = -1.f;
        screen.pMax.x = 1.f;
        screen.pMin.y = -1.f / frame;
        screen.pMax.y = 1.f / frame;
    }
    std::vector<Float> sw = parameters.GetFloatArray("screenwindow");
    if (!sw.empty()) {
        if (Options->fullscreen) {
            Warning("\"screenwindow\" is ignored in fullscreen mode");
        } else {
            if (sw.size() == 4) {
                screen.pMin.x = sw[0];
                screen.pMax.x = sw[1];
                screen.pMin.y = sw[2];
                screen.pMax.y = sw[3];
            } else {
                Error(loc, "\"screenwindow\" should have four values");
            }
        }
    }
    Float fov = parameters.GetOneFloat("fov", 90.);
    return alloc.new_object<PerspectiveCamera>(cameraBaseParameters, fov, screen,
                                               lensradius, focaldistance);
}

PBRT_CPU_GPU SampledSpectrum PerspectiveCamera::We(const Ray& ray,
                                                   SampledWavelengths& lambda,
                                                   Point2f* pRasterOut) const {
    // Check if ray is forward-facing with respect to the camera
    Float cosTheta = Dot(ray.d, RenderFromCamera(Vector3f(0, 0, 1), ray.time));
    if (cosTheta <= cosTotalWidth)
        return SampledSpectrum(0.);

    // Map ray $(\p{}, \w{})$ onto the raster grid
    Point3f pFocus = ray((lensRadius > 0 ? focalDistance : 1) / cosTheta);
    Point3f pCamera = CameraFromRender(pFocus, ray.time);
    Point3f pRaster = cameraFromRaster.ApplyInverse(pCamera);

    // Return raster position if requested
    if (pRasterOut)
        *pRasterOut = Point2f(pRaster.x, pRaster.y);

    // Return zero importance for out of bounds points
    Bounds2f sampleBounds = film.SampleBounds();
    if (!Inside(Point2f(pRaster.x, pRaster.y), sampleBounds))
        return SampledSpectrum(0.);

    // Compute lens area of perspective camera
    Float lensArea = lensRadius != 0 ? (Pi * Sqr(lensRadius)) : 1;

    // Return importance for point on image plane
    return SampledSpectrum(1 / (A * lensArea * Pow<4>(cosTheta)));
}

PBRT_CPU_GPU void PerspectiveCamera::PDF_We(const Ray& ray, Float* pdfPos,
                                            Float* pdfDir) const {
    // Return zero PDF values if ray direction is not front-facing
    Float cosTheta = Dot(ray.d, RenderFromCamera(Vector3f(0, 0, 1), ray.time));
    if (cosTheta <= cosTotalWidth) {
        *pdfPos = *pdfDir = 0;
        return;
    }

    // Map ray $(\p{}, \w{})$ onto the raster grid
    Point3f pFocus = ray((lensRadius > 0 ? focalDistance : 1) / cosTheta);
    Point3f pCamera = CameraFromRender(pFocus, ray.time);
    Point3f pRaster = cameraFromRaster.ApplyInverse(pCamera);

    // Return zero probability for out of bounds points
    Bounds2f sampleBounds = film.SampleBounds();
    if (!Inside(Point2f(pRaster.x, pRaster.y), sampleBounds)) {
        *pdfPos = *pdfDir = 0;
        return;
    }

    // Compute lens area  and return perspective camera probabilities
    Float lensArea = lensRadius != 0 ? (Pi * Sqr(lensRadius)) : 1;
    *pdfPos = 1 / lensArea;
    *pdfDir = 1 / (A * Pow<3>(cosTheta));
}

PBRT_CPU_GPU pstd::optional<CameraWiSample> PerspectiveCamera::SampleWi(
    const Interaction& ref, Point2f u, SampledWavelengths& lambda) const {
    // Uniformly sample a lens interaction _lensIntr_
    Point2f pLens = lensRadius * SampleUniformDiskConcentric(u);
    Point3f pLensRender = RenderFromCamera(Point3f(pLens.x, pLens.y, 0), ref.time);
    Normal3f n = Normal3f(RenderFromCamera(Vector3f(0, 0, 1), ref.time));
    Interaction lensIntr(pLensRender, n, ref.time, medium);

    // Find incident direction to camera _wi_ at _ref_
    Vector3f wi = lensIntr.p() - ref.p();
    Float dist = Length(wi);
    wi /= dist;

    // Compute PDF for importance arriving at _ref_
    Float lensArea = lensRadius != 0 ? (Pi * Sqr(lensRadius)) : 1;
    Float pdf = Sqr(dist) / (AbsDot(lensIntr.n, wi) * lensArea);

    // Compute importance and return _CameraWiSample_
    Point2f pRaster;
    SampledSpectrum Wi = We(lensIntr.SpawnRay(-wi), lambda, &pRaster);
    if (!Wi)
        return {};
    return CameraWiSample(Wi, wi, pdf, pRaster, ref, lensIntr);
}

// SphericalCamera Method Definitions
PBRT_CPU_GPU pstd::optional<CameraRay> SphericalCamera::GenerateRay(CameraSample sample,
    SampledWavelengths& lambda) const {
    // Compute spherical camera ray direction
    Point2f uv(sample.pFilm.x / film.FullResolution().x,
               sample.pFilm.y / film.FullResolution().y);
    Vector3f dir;
    if (mapping == EquiRectangular) {
        // Compute ray direction using equirectangular mapping
        Float theta = Pi * uv[1], phi = 2 * Pi * uv[0];
        dir = SphericalDirection(std::sin(theta), std::cos(theta), phi);
    } else {
        // Compute ray direction using equal area mapping
        uv = WrapEqualAreaSquare(uv);
        dir = EqualAreaSquareToSphere(uv);
    }
    pstd::swap(dir.y, dir.z);

    Ray ray(Point3f(0, 0, 0), dir, SampleTime(sample.time), medium);
    return CameraRay{RenderFromCamera(ray)};
}

SphericalCamera* SphericalCamera::Create(const ParameterDictionary& parameters,
                                         const CameraTransform& cameraTransform,
                                         Film film, Medium medium, const FileLoc* loc,
                                         Allocator alloc) {
    CameraBaseParameters cameraBaseParameters(cameraTransform, film, medium, parameters,
                                              loc);

    Float lensradius = parameters.GetOneFloat("lensradius", 0.f);
    Float focaldistance = parameters.GetOneFloat("focaldistance", 1e30f);
    Float frame =
        parameters.GetOneFloat("frameaspectratio", Float(film.FullResolution().x) /
                                                   Float(film.FullResolution().y));
    Bounds2f screen;
    if (frame > 1.f) {
        screen.pMin.x = -frame;
        screen.pMax.x = frame;
        screen.pMin.y = -1.f;
        screen.pMax.y = 1.f;
    } else {
        screen.pMin.x = -1.f;
        screen.pMax.x = 1.f;
        screen.pMin.y = -1.f / frame;
        screen.pMax.y = 1.f / frame;
    }
    std::vector<Float> sw = parameters.GetFloatArray("screenwindow");
    if (!sw.empty()) {
        if (Options->fullscreen) {
            Warning("\"screenwindow\" is ignored in fullscreen mode");
        } else {
            if (sw.size() == 4) {
                screen.pMin.x = sw[0];
                screen.pMax.x = sw[1];
                screen.pMin.y = sw[2];
                screen.pMax.y = sw[3];
            } else {
                Error(loc, "\"screenwindow\" should have four values");
            }
        }
    }
    (void)lensradius; // don't need this
    (void)focaldistance; // don't need this

    std::string m = parameters.GetOneString("mapping", "equalarea");
    Mapping mapping;
    if (m == "equalarea")
        mapping = EqualArea;
    else if (m == "equirectangular")
        mapping = EquiRectangular;
    else
        ErrorExit(loc,
                  "%s: unknown mapping for spherical camera. (Must be "
                  "\"equalarea\" or \"equirectangular\".)",
                  m);

    return alloc.new_object<SphericalCamera>(cameraBaseParameters, mapping);
}

std::string SphericalCamera::ToString() const {
    return StringPrintf("[ SphericalCamera %s mapping: %s ]", CameraBase::ToString(),
                        mapping == EquiRectangular ? "EquiRectangular" : "EqualArea");
}

// RealisticCamera Method Definitions
RealisticCamera::RealisticCamera(CameraBaseParameters baseParameters,
                                 std::vector<Float>& lensParameters, Float focusDistance,
                                 Float setApertureDiameter, int flareSamples, bool sellmeier,
                                 Image apertureImage,
                                 Allocator alloc)
    : CameraBase(baseParameters),
      elementInterfaces(alloc),
      exitPupilBounds(alloc),
      apertureImage(std::move(apertureImage)),
      flareSamples(flareSamples),
    sellmeier(sellmeier) {
    // Compute film's physical extent
    Float aspect = (Float)film.FullResolution().y / (Float)film.FullResolution().x;
    Float diagonal = film.Diagonal();
    Float x = std::sqrt(Sqr(diagonal) / (1 + Sqr(aspect)));
    Float y = aspect * x;
    physicalExtent = Bounds2f(Point2f(-x / 2, -y / 2), Point2f(x / 2, y / 2));

    // Initialize _elementInterfaces_ for camera
    for (size_t i = 0; i < lensParameters.size();) {
        LensElementInterface element{};
        element.curvatureRadiusX = lensParameters[i] / 1000;
        element.curvatureRadiusY = lensParameters[i + 1] / 1000;
        element.thickness = lensParameters[i + 2] / 1000;
        element.apertureRadius = lensParameters[i + 3] / 1000 / 2;
        element.eta = lensParameters[i + 4];
        // Read Sellmeier coefficients (B1, B2, B3, C1, C2, C3)
        element.B1 = lensParameters[i + 5];
        element.B2 = lensParameters[i + 6];
        element.B3 = lensParameters[i + 7];
        element.C1 = lensParameters[i + 8];
        element.C2 = lensParameters[i + 9];
        element.C3 = lensParameters[i + 10];

        // Aperture stop handling
        if (element.curvatureRadiusX == 0  && element.curvatureRadiusY == 0) {
            // Aperture stop
            setApertureDiameter /= 1000;
            if (setApertureDiameter > element.apertureRadius * 2)
                Warning("Aperture diameter %f is greater than maximum possible %f. "
                        "Clamping it.", setApertureDiameter, element.apertureRadius * 2);
            else
                element.apertureRadius = setApertureDiameter / 2;
        }

        elementInterfaces.push_back(element);
        i += 11; // Move to next element
    }

    // Compute lens--film distance for given focus distance
    elementInterfaces.back().thickness = FocusThickLens(focusDistance); //TODO breaks for anamorphic lenses
    LOG_VERBOSE("Lens film distance for focus distance %f: %f", focusDistance,
                elementInterfaces.back().thickness);

    // Compute exit pupil bounds at sampled points on the film
    int nSamples = 64;
    exitPupilBounds.resize(nSamples);
    ParallelFor(0, nSamples, [&](int i) {
        Float r0 = (Float)i / nSamples * film.Diagonal() / 2;
        Float r1 = (Float)(i + 1) / nSamples * film.Diagonal() / 2;
        exitPupilBounds[i] = BoundExitPupil(r0, r1);
    });

    // Compute minimum differentials for _RealisticCamera_
    FindMinimumDifferentials(this);
}

PBRT_CPU_GPU Float RealisticCamera::TraceLensesFromFilm(const Ray &rCamera, Ray *rOut) const {
    Float elementZ = 0, weight = 1;
    // Transform _rCamera_ from camera to lens system space
    Ray rLens(Point3f(rCamera.o.x, rCamera.o.y, -rCamera.o.z),
              Vector3f(rCamera.d.x, rCamera.d.y, -rCamera.d.z), rCamera.time);

    for (int i = elementInterfaces.size() - 1; i >= 0; --i) {
        const LensElementInterface &element = elementInterfaces[i];
        // Update ray from film accounting for interaction with _element_
        elementZ -= element.thickness;
        // Compute intersection of ray with lens element
        Float t;
        Normal3f n;
        bool isStop = (element.curvatureRadiusX == 0 && element.curvatureRadiusY == 0);
        if (isStop) {
            // Compute _t_ at plane of aperture stop
            t = (elementZ - rLens.o.z) / rLens.d.z;
            if (t < 0)
                return 0;

        } else {
            // Intersect ray with element to compute _t_ and _n_
            Float radius = element.curvatureRadiusX;
            Float zCenter = elementZ + element.curvatureRadiusX;
            if (!IntersectSphericalElement(radius, zCenter, rLens, &t, &n))
                return 0;
        }
        DCHECK_GE(t, 0);

        // Test intersection point against element aperture
        Point3f pHit = rLens(t);
        if (isStop && apertureImage) {
            // Check intersection point against _apertureImage_
            Point2f uv((pHit.x / element.apertureRadius + 1) / 2,
                       (pHit.y / element.apertureRadius + 1) / 2);
            weight = apertureImage.BilerpChannel(uv, 0, WrapMode::Black);
            if (weight == 0)
                return 0;

        } else {
            // Check intersection point against spherical aperture
            if (Sqr(pHit.x) + Sqr(pHit.y) > Sqr(element.apertureRadius))
                return 0;
        }
        rLens.o = pHit;

        // Update ray path for element interface interaction
        if (!isStop) {
            Vector3f w;
            Float eta_i = element.eta;
            Float eta_t = (i > 0 && elementInterfaces[i - 1].eta != 0)
                              ? elementInterfaces[i - 1].eta
                              : 1;
            if (!Refract(Normalize(-rLens.d), n, eta_t / eta_i, nullptr, &w))
                return 0;
            rLens.d = w;
        }
    }
    // Transform lens system space ray back to camera space
    if (rOut)
        *rOut = Ray(Point3f(rLens.o.x, rLens.o.y, -rLens.o.z),
                    Vector3f(rLens.d.x, rLens.d.y, -rLens.d.z), rLens.time);

    return weight;
}

void RealisticCamera::ComputeCardinalPoints(Ray rIn, Ray rOut, Float *pz, Float *fz) {
    Float tf = -rOut.o.x / rOut.d.x;
    *fz = -rOut(tf).z;
    Float tp = (rIn.o.x - rOut.o.x) / rOut.d.x;
    *pz = -rOut(tp).z;
}

void RealisticCamera::ComputeThickLensApproximation(Float pz[2], Float fz[2]) const {
    // Find height $x$ from optical axis for parallel rays
    Float x = .001f * film.Diagonal();

    // Compute cardinal points for film side of lens system
    Ray rScene(Point3f(x, 0, LensFrontZ() + 1), Vector3f(0, 0, -1));
    Ray rFilm;
    if (!TraceLensesFromScene(rScene, &rFilm))
        ErrorExit("Unable to trace ray from scene to film for thick lens "
                  "approximation. Is aperture stop extremely small?");
    ComputeCardinalPoints(rScene, rFilm, &pz[0], &fz[0]);

    // Compute cardinal points for scene side of lens system
    rFilm = Ray(Point3f(x, 0, LensRearZ() - 1), Vector3f(0, 0, 1));
    if (TraceLensesFromFilm(rFilm, &rScene) == 0)
        ErrorExit("Unable to trace ray from film to scene for thick lens "
                  "approximation. Is aperture stop extremely small?");
    ComputeCardinalPoints(rFilm, rScene, &pz[1], &fz[1]);
}

Float RealisticCamera::FocusThickLens(Float focusDistance) {
    Float pz[2], fz[2];
    ComputeThickLensApproximation(pz, fz);
    LOG_VERBOSE("Cardinal points: p' = %f f' = %f, p = %f f = %f.\n", pz[0], fz[0], pz[1],
                fz[1]);
    LOG_VERBOSE("Effective focal length %f\n", fz[0] - pz[0]);
    // Compute translation of lens, _delta_, to focus at _focusDistance_
    Float f = fz[0] - pz[0];
    Float z = -focusDistance;
    Float c = (pz[1] - z - pz[0]) * (pz[1] - z - 4 * f - pz[0]);
    if (c <= 0)
        ErrorExit("Coefficient must be positive. It looks focusDistance %f "
                  " is too short for a given lenses configuration",
                  focusDistance);
    Float delta = (pz[1] - z + pz[0] - std::sqrt(c)) / 2;

    return elementInterfaces.back().thickness + delta;
}

Bounds2f RealisticCamera::BoundExitPupil(Float filmX0, Float filmX1) const {
    Bounds2f pupilBounds;
    // Sample a collection of points on the rear lens to find exit pupil
    const int nSamples = 1024 * 1024;
    // Compute bounding box of projection of rear element on sampling plane
    Float rearRadius = RearElementRadius();
    Bounds2f projRearBounds(Point2f(-1.5f * rearRadius, -1.5f * rearRadius),
                            Point2f(1.5f * rearRadius, 1.5f * rearRadius));

    for (int i = 0; i < nSamples; ++i) {
        // Find location of sample points on $x$ segment and rear lens element
        Point3f pFilm(Lerp((i + 0.5f) / nSamples, filmX0, filmX1), 0, 0);
        Float u[2] = {RadicalInverse(0, i), RadicalInverse(1, i)};
        Point3f pRear(Lerp(u[0], projRearBounds.pMin.x, projRearBounds.pMax.x),
                      Lerp(u[1], projRearBounds.pMin.y, projRearBounds.pMax.y),
                      LensRearZ());

        // Expand pupil bounds if ray makes it through the lens system
        if (!Inside(Point2f(pRear.x, pRear.y), pupilBounds) &&
            TraceLensesFromFilm(Ray(pFilm, pRear - pFilm), nullptr))
            pupilBounds = Union(pupilBounds, Point2f(pRear.x, pRear.y));
    }

    // Return degenerate bounds if no rays made it through the lens system
    if (pupilBounds.IsDegenerate()) {
        LOG_VERBOSE("Unable to find exit pupil in x = [%f,%f] on film.", filmX0, filmX1);
        return pupilBounds;
    }

    // Expand bounds to account for sample spacing
    pupilBounds =
        Expand(pupilBounds, 2 * Length(projRearBounds.Diagonal()) / std::sqrt(nSamples));

    return pupilBounds;
}

PBRT_CPU_GPU pstd::optional<ExitPupilSample> RealisticCamera::SampleExitPupil(Point2f pFilm,
                                                                 Point2f uLens) const {
    // Find exit pupil bound for sample distance from film center
    Float rFilm = std::sqrt(Sqr(pFilm.x) + Sqr(pFilm.y));
    int rIndex = rFilm / (film.Diagonal() / 2) * exitPupilBounds.size();
    rIndex = std::min<int>(exitPupilBounds.size() - 1, rIndex);
    Bounds2f pupilBounds = exitPupilBounds[rIndex];
    if (pupilBounds.IsDegenerate())
        return {};

    // Generate sample point inside exit pupil bound
    Point2f pLens = pupilBounds.Lerp(uLens);
    Float pdf = 1 / pupilBounds.Area();

    // Return sample point rotated by angle of _pFilm_ with $+x$ axis
    Float sinTheta = (rFilm != 0) ? pFilm.y / rFilm : 0;
    Float cosTheta = (rFilm != 0) ? pFilm.x / rFilm : 1;
    Point3f pPupil(cosTheta * pLens.x - sinTheta * pLens.y,
                   sinTheta * pLens.x + cosTheta * pLens.y, LensRearZ());
    return ExitPupilSample{pPupil, pdf};
}

PBRT_CPU_GPU pstd::optional<CameraRay> RealisticCamera::GenerateRay(CameraSample sample,
                                                       SampledWavelengths &lambda) const {
    // Find point on film, _pFilm_, corresponding to _sample.pFilm_
    Point2f s(sample.pFilm.x / film.FullResolution().x,
              sample.pFilm.y / film.FullResolution().y);
    Point2f pFilm2 = physicalExtent.Lerp(s);
    Point3f pFilm(-pFilm2.x, pFilm2.y, 0);

    // Trace ray from _pFilm_ through lens system
    pstd::optional<ExitPupilSample> eps =
        SampleExitPupil(Point2f(pFilm.x, pFilm.y), sample.pLens);
    if (!eps)
        return {};
    Ray rFilm(pFilm, eps->pPupil - pFilm);
    Ray ray;
    Float weight = TraceLensesFromFilm(rFilm, &ray);
    if (weight == 0)
        return {};

    // Finish initialization of _RealisticCamera_ ray
    ray.time = SampleTime(sample.time);
    ray.medium = medium;
    ray = RenderFromCamera(ray);
    ray.d = Normalize(ray.d);

    // Compute weighting for _RealisticCamera_ ray
    Float cosTheta = Normalize(rFilm.d).z;
    weight *= Pow<4>(cosTheta) / (eps->pdf * Sqr(LensRearZ()));

    return CameraRay{ray, SampledSpectrum(weight)};
}

STAT_PERCENT("Camera/Rays vignetted by lens system", vignettedRays, totalRays);

std::string RealisticCamera::LensElementInterface::ToString() const {
    return StringPrintf("[ LensElementInterface curvatureRadius: %f thickness: %f "
                        "eta: %f apertureRadius: %f ]",
                        curvatureRadiusX, thickness, eta, apertureRadius);
}

PBRT_CPU_GPU Float RealisticCamera::TraceLensesFromScene(const Ray &rCamera, Ray *rOut) const {
    Float elementZ = -LensFrontZ();
    // Transform _rCamera_ from camera to lens system space
    const Transform LensFromCamera = Scale(1, 1, -1);
    Ray rLens = LensFromCamera(rCamera);
    for (size_t i = 0; i < elementInterfaces.size(); ++i) {
        const LensElementInterface &element = elementInterfaces[i];
        // Compute intersection of ray with lens element
        Float t;
        Normal3f n;
        bool isStop = (element.curvatureRadiusX == 0 && element.curvatureRadiusY == 0);
        if (isStop) {
            t = (elementZ - rLens.o.z) / rLens.d.z;
            if (t < 0)
                return 0;
        } else {
            Float radius = element.curvatureRadiusX;
            Float zCenter = elementZ + element.curvatureRadiusX;
            if (!IntersectSphericalElement(radius, zCenter, rLens, &t, &n))
                return 0;
        }

        // Test intersection point against element aperture
        // Don't worry about the aperture image here.
        Point3f pHit = rLens(t);
        Float r2 = pHit.x * pHit.x + pHit.y * pHit.y;
        if (r2 > element.apertureRadius * element.apertureRadius)
            return 0;
        rLens.o = pHit;

        // Update ray path for from-scene element interface interaction
        if (!isStop) {
            Vector3f wt;
            Float eta_i = (i == 0 || elementInterfaces[i - 1].eta == 0)
                              ? 1
                              : elementInterfaces[i - 1].eta;
            Float eta_t = (elementInterfaces[i].eta != 0) ? elementInterfaces[i].eta : 1;
            if (!Refract(Normalize(-rLens.d), n, eta_t / eta_i, nullptr, &wt))
                return 0;
            rLens.d = wt;
        }
        elementZ += element.thickness;
    }
    // Transform _rLens_ from lens system space back to camera space
    if (rOut)
        *rOut = Ray(Point3f(rLens.o.x, rLens.o.y, -rLens.o.z),
                    Vector3f(rLens.d.x, rLens.d.y, -rLens.d.z), rLens.time);
    return 1;
}

void RealisticCamera::DrawLensSystem() const {
    Float sumz = -LensFrontZ();
    Float z = sumz;
    for (size_t i = 0; i < elementInterfaces.size(); ++i) {
        const LensElementInterface &element = elementInterfaces[i];
        Float r = element.curvatureRadiusX;
        if (r == 0) {
            // stop
            printf("{Thick, Line[{{%f, %f}, {%f, %f}}], ", z, element.apertureRadius, z,
                   2 * element.apertureRadius);
            printf("Line[{{%f, %f}, {%f, %f}}]}, ", z, -element.apertureRadius, z,
                   -2 * element.apertureRadius);
        } else {
            Float theta = std::abs(SafeASin(element.apertureRadius / r));
            if (r > 0) {
                // convex as seen from front of lens
                Float t0 = Pi - theta;
                Float t1 = Pi + theta;
                printf("Circle[{%f, 0}, %f, {%f, %f}], ", z + r, r, t0, t1);
            } else {
                // concave as seen from front of lens
                Float t0 = -theta;
                Float t1 = theta;
                printf("Circle[{%f, 0}, %f, {%f, %f}], ", z + r, -r, t0, t1);
            }
            if (element.eta != 0 && element.eta != 1) {
                // connect top/bottom to next element
                CHECK_LT(i + 1, elementInterfaces.size());
                Float nextApertureRadius = elementInterfaces[i + 1].apertureRadius;
                Float h = std::max(element.apertureRadius, nextApertureRadius);
                Float hlow = std::min(element.apertureRadius, nextApertureRadius);

                Float zp0, zp1;
                if (r > 0) {
                    zp0 = z + element.curvatureRadiusX -
                          element.apertureRadius / std::tan(theta);
                } else {
                    zp0 = z + element.curvatureRadiusX +
                          element.apertureRadius / std::tan(theta);
                }

                Float nextCurvatureRadius = elementInterfaces[i + 1].curvatureRadiusX;
                Float nextTheta =
                    std::abs(SafeASin(nextApertureRadius / nextCurvatureRadius));
                if (nextCurvatureRadius > 0) {
                    zp1 = z + element.thickness + nextCurvatureRadius -
                          nextApertureRadius / std::tan(nextTheta);
                } else {
                    zp1 = z + element.thickness + nextCurvatureRadius +
                          nextApertureRadius / std::tan(nextTheta);
                }

                // Connect tops
                printf("Line[{{%f, %f}, {%f, %f}}], ", zp0, h, zp1, h);
                printf("Line[{{%f, %f}, {%f, %f}}], ", zp0, -h, zp1, -h);

                // vertical lines when needed to close up the element profile
                if (element.apertureRadius < nextApertureRadius) {
                    printf("Line[{{%f, %f}, {%f, %f}}], ", zp0, h, zp0, hlow);
                    printf("Line[{{%f, %f}, {%f, %f}}], ", zp0, -h, zp0, -hlow);
                } else if (element.apertureRadius > nextApertureRadius) {
                    printf("Line[{{%f, %f}, {%f, %f}}], ", zp1, h, zp1, hlow);
                    printf("Line[{{%f, %f}, {%f, %f}}], ", zp1, -h, zp1, -hlow);
                }
            }
        }
        z += element.thickness;
    }

    // 24mm height for 35mm film
    printf("Line[{{0, -.012}, {0, .012}}], ");
    // optical axis
    printf("Line[{{0, 0}, {%f, 0}}] ", 1.2f * sumz);
}

void RealisticCamera::DrawRayPathFromFilm(const Ray &r, bool arrow,
                                          bool toOpticalIntercept) const {
    Float elementZ = 0;
    // Transform _ray_ from camera to lens system space
    static const Transform LensFromCamera = Scale(1, 1, -1);
    Ray ray = LensFromCamera(r);
    printf("{ ");
    if (TraceLensesFromFilm(r, nullptr) == 0) {
        printf("Dashed, RGBColor[.8, .5, .5]");
    } else
        printf("RGBColor[.5, .5, .8]");

    for (int i = elementInterfaces.size() - 1; i >= 0; --i) {
        const LensElementInterface &element = elementInterfaces[i];
        elementZ -= element.thickness;
        bool isStop = (element.curvatureRadiusX == 0 && element.curvatureRadiusY);
        // Compute intersection of ray with lens element
        Float t;
        Normal3f n;
        if (isStop)
            t = -(ray.o.z - elementZ) / ray.d.z;
        else {
            Float radius = element.curvatureRadiusX;
            Float zCenter = elementZ + element.curvatureRadiusX;
            if (!IntersectSphericalElement(radius, zCenter, ray, &t, &n))
                goto done;
        }
        CHECK_GE(t, 0);

        printf(", Line[{{%f, %f}, {%f, %f}}]", ray.o.z, ray.o.x, ray(t).z, ray(t).x);

        // Test intersection point against element aperture
        Point3f pHit = ray(t);
        Float r2 = pHit.x * pHit.x + pHit.y * pHit.y;
        Float apertureRadius2 = element.apertureRadius * element.apertureRadius;
        if (r2 > apertureRadius2)
            goto done;
        ray.o = pHit;

        // Update ray path for element interface interaction
        if (!isStop) {
            Vector3f wt;
            Float eta_i = element.eta;
            Float eta_t = (i > 0 && elementInterfaces[i - 1].eta != 0)
                              ? elementInterfaces[i - 1].eta
                              : 1;
            if (!Refract(Normalize(-ray.d), n, eta_t / eta_i, nullptr, &wt))
                goto done;
            ray.d = wt;
        }
    }

    ray.d = Normalize(ray.d);
    {
        Float ta = std::abs(elementZ / 4);
        if (toOpticalIntercept) {
            ta = -ray.o.x / ray.d.x;
            printf(", Point[{%f, %f}]", ray(ta).z, ray(ta).x);
        }
        printf(", %s[{{%f, %f}, {%f, %f}}]", arrow ? "Arrow" : "Line", ray.o.z, ray.o.x,
               ray(ta).z, ray(ta).x);

        // overdraw the optical axis if needed...
        if (toOpticalIntercept)
            printf(", Line[{{%f, 0}, {%f, 0}}]", ray.o.z, ray(ta).z * 1.05f);
    }

    done:
        printf("}");
}

void RealisticCamera::DrawRayPathFromScene(const Ray &r, bool arrow,
                                           bool toOpticalIntercept) const {
    Float elementZ = LensFrontZ() * -1;

    // Transform _ray_ from camera to lens system space
    static const Transform LensFromCamera = Scale(1, 1, -1);
    Ray ray = LensFromCamera(r);
    for (size_t i = 0; i < elementInterfaces.size(); ++i) {
        const LensElementInterface &element = elementInterfaces[i];
        bool isStop = (element.curvatureRadiusX == 0 && element.curvatureRadiusY);
        // Compute intersection of ray with lens element
        Float t;
        Normal3f n;
        if (isStop)
            t = -(ray.o.z - elementZ) / ray.d.z;
        else {
            Float radius = element.curvatureRadiusX;
            Float zCenter = elementZ + element.curvatureRadiusX;
            if (!IntersectSphericalElement(radius, zCenter, ray, &t, &n))
                return;
        }
        CHECK_GE(t, 0.f);

        printf("Line[{{%f, %f}, {%f, %f}}],", ray.o.z, ray.o.x, ray(t).z, ray(t).x);

        // Test intersection point against element aperture
        Point3f pHit = ray(t);
        Float r2 = pHit.x * pHit.x + pHit.y * pHit.y;
        Float apertureRadius2 = element.apertureRadius * element.apertureRadius;
        if (r2 > apertureRadius2)
            return;
        ray.o = pHit;

        // Update ray path for from-scene element interface interaction
        if (!isStop) {
            Vector3f wt;
            Float eta_i = (i == 0 || elementInterfaces[i - 1].eta == 0.f)
                              ? 1.f
                              : elementInterfaces[i - 1].eta;
            Float eta_t =
                (elementInterfaces[i].eta != 0.f) ? elementInterfaces[i].eta : 1.f;
            if (!Refract(Normalize(-ray.d), n, eta_t / eta_i, nullptr, &wt))
                return;
            ray.d = wt;
        }
        elementZ += element.thickness;
    }

    // go to the film plane by default
    {
        Float ta = -ray.o.z / ray.d.z;
        if (toOpticalIntercept) {
            ta = -ray.o.x / ray.d.x;
            printf("Point[{%f, %f}], ", ray(ta).z, ray(ta).x);
        }
        printf("%s[{{%f, %f}, {%f, %f}}]", arrow ? "Arrow" : "Line", ray.o.z, ray.o.x,
               ray(ta).z, ray(ta).x);
    }
}

void RealisticCamera::RenderExitPupil(Float sx, Float sy, const char *filename) const {
    Point3f pFilm(sx, sy, 0);

    const int nSamples = 2048;
    Image image(PixelFormat::Float, {nSamples, nSamples}, {"Y"});

    for (int y = 0; y < nSamples; ++y) {
        Float fy = (Float)y / (Float)(nSamples - 1);
        Float ly = Lerp(fy, -RearElementRadius(), RearElementRadius());
        for (int x = 0; x < nSamples; ++x) {
            Float fx = (Float)x / (Float)(nSamples - 1);
            Float lx = Lerp(fx, -RearElementRadius(), RearElementRadius());

            Point3f pRear(lx, ly, LensRearZ());

            if (lx * lx + ly * ly > RearElementRadius() * RearElementRadius())
                image.SetChannel({x, y}, 0, 1.);
            else if (TraceLensesFromFilm(Ray(pFilm, pRear - pFilm), nullptr))
                image.SetChannel({x, y}, 0, 0.5);
            else
                image.SetChannel({x, y}, 0, 0.);
        }
    }

    image.Write(filename);
}

void RealisticCamera::TestExitPupilBounds() const {
    Float filmDiagonal = film.Diagonal();

    static RNG rng;

    Float u = rng.Uniform<Float>();
    Point3f pFilm(u * filmDiagonal / 2, 0, 0);

    Float r = pFilm.x / (filmDiagonal / 2);
    int pupilIndex = std::min<int>(exitPupilBounds.size() - 1,
                                   pstd::floor(r * (exitPupilBounds.size() - 1)));
    Bounds2f pupilBounds = exitPupilBounds[pupilIndex];
    if (pupilIndex + 1 < (int)exitPupilBounds.size())
        pupilBounds = Union(pupilBounds, exitPupilBounds[pupilIndex + 1]);

    // Now, randomly pick points on the aperture and see if any are outside
    // of pupil bounds...
    for (int i = 0; i < 1000; ++i) {
        Point2f u2{rng.Uniform<Float>(), rng.Uniform<Float>()};
        Point2f pd = SampleUniformDiskConcentric(u2);
        pd *= RearElementRadius();

        Ray testRay(pFilm, Point3f(pd.x, pd.y, 0.f) - pFilm);
        Ray testOut;
        if (!TraceLensesFromFilm(testRay, &testOut))
            continue;

        if (!Inside(pd, pupilBounds)) {
            fprintf(stderr,
                    "Aha! (%f,%f) went through, but outside bounds (%f,%f) - "
                    "(%f,%f)\n",
                    pd.x, pd.y, pupilBounds.pMin[0], pupilBounds.pMin[1],
                    pupilBounds.pMax[0], pupilBounds.pMax[1]);
            RenderExitPupil(
                (Float)pupilIndex / exitPupilBounds.size() * filmDiagonal / 2.f, 0.f,
                "low.exr");
            RenderExitPupil(
                (Float)(pupilIndex + 1) / exitPupilBounds.size() * filmDiagonal / 2.f,
                0.f, "high.exr");
            RenderExitPupil(pFilm.x, 0.f, "mid.exr");
            exit(0);
        }
    }
    fprintf(stderr, ".");
}

std::string RealisticCamera::ToString() const {
    return StringPrintf(
        "[ RealisticCamera %s elementInterfaces: %s exitPupilBounds: %s ]",
        CameraBase::ToString(), elementInterfaces, exitPupilBounds);
}

RealisticCamera *RealisticCamera::Create(const ParameterDictionary &parameters,
                                         const CameraTransform &cameraTransform,
                                         Film film, Medium medium, const FileLoc *loc,
                                         Allocator alloc) {
    CameraBaseParameters cameraBaseParameters(cameraTransform, film, medium, parameters,
                                              loc);

    // Realistic camera-specific parameters
    std::string lensFile = ResolveFilename(parameters.GetOneString("lensfile", ""));
    Float apertureDiameter = parameters.GetOneFloat("aperturediameter", 1.0);
    Float focusDistance = parameters.GetOneFloat("focusdistance", 10.0);
    int flareSamples = parameters.GetOneInt("flaresamples", 1000000000);
    bool sellmeier = parameters.GetOneBool("sellmeier", false);

    if (lensFile.empty()) {
        Error(loc, "No lens description file supplied!");
        return nullptr;
    }
    // Load element data from lens description file
    std::vector<Float> lensParameters = ReadFloatFile(lensFile);
    if (lensParameters.empty()) {
        Error(loc, "Error reading lens specification file \"%s\".", lensFile);
        return nullptr;
    }
    //if (lensParameters.size() % 4 != 0) {
    //    Error(loc, "%s: excess values in lens specification file; " "must be multiple-of-four values, read %d.", lensFile, (int)lensParameters.size());
    //    return nullptr;
    //}

    int builtinRes = 256;
    auto rasterize = [&](pstd::span<const Point2f> vert) {
        Image image(PixelFormat::Float, {builtinRes, builtinRes}, {"Y"}, nullptr, alloc);

        for (int y = 0; y < image.Resolution().y; ++y)
            for (int x = 0; x < image.Resolution().x; ++x) {
                Point2f p(-1 + 2 * (x + 0.5f) / image.Resolution().x,
                          -1 + 2 * (y + 0.5f) / image.Resolution().y);
                int windingNumber = 0;
                // Test against edges
                for (int i = 0; i < vert.size(); ++i) {
                    int i1 = (i + 1) % vert.size();
                    Float e = (p[0] - vert[i][0]) * (vert[i1][1] - vert[i][1]) -
                              (p[1] - vert[i][1]) * (vert[i1][0] - vert[i][0]);
                    if (vert[i].y <= p.y) {
                        if (vert[i1].y > p.y && e > 0)
                            ++windingNumber;
                    } else if (vert[i1].y <= p.y && e < 0)
                        --windingNumber;
                }

                image.SetChannel({x, y}, 0, windingNumber == 0 ? 0.f : 1.f);
            }

        return image;
    };

    std::string apertureName = parameters.GetOneString("aperture", "");
    Image apertureImage(alloc);
    if (!apertureName.empty()) {
        // built-in diaphragm shapes
        if (apertureName == "gaussian") {
            apertureImage = Image(PixelFormat::Float, {builtinRes, builtinRes}, {"Y"},
                                  nullptr, alloc);
            for (int y = 0; y < apertureImage.Resolution().y; ++y)
                for (int x = 0; x < apertureImage.Resolution().x; ++x) {
                    Point2f uv(-1 + 2 * (x + 0.5f) / apertureImage.Resolution().x,
                               -1 + 2 * (y + 0.5f) / apertureImage.Resolution().y);
                    Float r2 = Sqr(uv.x) + Sqr(uv.y);
                    Float sigma2 = 1;
                    Float v = std::max<Float>(
                        0, std::exp(-r2 / sigma2) - std::exp(-1 / sigma2));
                    apertureImage.SetChannel({x, y}, 0, v);
                }
        } else if (apertureName == "square") {
            apertureImage = Image(PixelFormat::Float, {builtinRes, builtinRes}, {"Y"},
                                  nullptr, alloc);
            for (int y = .25 * builtinRes; y < .75 * builtinRes; ++y)
                for (int x = .25 * builtinRes; x < .75 * builtinRes; ++x)
                    apertureImage.SetChannel({x, y}, 0, 4.f);
        } else if (apertureName == "pentagon") {
            // https://mathworld.wolfram.com/RegularPentagon.html
            Float c1 = (std::sqrt(5.f) - 1) / 4;
            Float c2 = (std::sqrt(5.f) + 1) / 4;
            Float s1 = std::sqrt(10.f + 2.f * std::sqrt(5.f)) / 4;
            Float s2 = std::sqrt(10.f - 2.f * std::sqrt(5.f)) / 4;
            // Vertices in CW order.
            Point2f vert[5] = {Point2f(0, 1), {s1, c1}, {s2, -c2}, {-s2, -c2}, {-s1, c1}};
            // Scale down slightly
            for (int i = 0; i < 5; ++i)
                vert[i] *= .8f;
            apertureImage = rasterize(vert);
        } else if (apertureName == "star") {
            // 5-sided. Vertices are two pentagons--inner and outer radius
            pstd::array<Point2f, 10> vert;
            for (int i = 0; i < 10; ++i) {
                // inner radius: https://math.stackexchange.com/a/2136996
                Float r =
                    (i & 1) ? 1.f : (std::cos(Radians(72.f)) / std::cos(Radians(36.f)));
                vert[i] = Point2f(r * std::cos(Pi * i / 5.f), r * std::sin(Pi * i / 5.f));
            }
            std::reverse(vert.begin(), vert.end());
            apertureImage = rasterize(vert);
        } else {
            ImageAndMetadata im = Image::Read(ResolveFilename(apertureName), alloc);
            apertureImage = std::move(im.image);
            if (apertureImage.NChannels() > 1) {
                ImageChannelDesc rgbDesc = apertureImage.GetChannelDesc({"R", "G", "B"});
                if (!rgbDesc)
                    ErrorExit("%s: didn't find R, G, B channels to average for "
                              "aperture image.",
                              apertureName);

                Image mono(PixelFormat::Float, apertureImage.Resolution(), {"Y"}, nullptr,
                           alloc);
                for (int y = 0; y < mono.Resolution().y; ++y)
                    for (int x = 0; x < mono.Resolution().x; ++x) {
                        Float avg = apertureImage.GetChannels({x, y}, rgbDesc).Average();
                        mono.SetChannel({x, y}, 0, avg);
                    }

                apertureImage = std::move(mono);
            }
        }

        if (apertureImage) {
            apertureImage.FlipY();

            // Normalize it so that brightness matches a circular aperture
            Float sum = 0;
            for (int y = 0; y < apertureImage.Resolution().y; ++y)
                for (int x = 0; x < apertureImage.Resolution().x; ++x)
                    sum += apertureImage.GetChannel({x, y}, 0);
            Float avg =
                sum / (apertureImage.Resolution().x * apertureImage.Resolution().y);

            Float scale = (Pi / 4) / avg;
            for (int y = 0; y < apertureImage.Resolution().y; ++y)
                for (int x = 0; x < apertureImage.Resolution().x; ++x)
                    apertureImage.SetChannel({x, y}, 0,
                                             apertureImage.GetChannel({x, y}, 0) * scale);
        }
    }

    return alloc.new_object<RealisticCamera>(cameraBaseParameters, lensParameters,
                                             focusDistance, apertureDiameter, flareSamples, sellmeier,
                                             std::move(apertureImage), alloc);
}

PBRT_CPU_GPU inline Float RealisticCamera::Sellmeier(
const Float wavelengthNm,
const Float B1, const Float B2, const Float B3,
const Float C1, const Float C2, const Float C3) const
{
    // Convert wavelength from nm to μm for Sellmeier formula
    const Float lambda = wavelengthNm / 1000.0f;
    const Float lambda2 = lambda * lambda;
    
    // Sellmeier equation: n^2(lam) = 1 + B1 * lam^2/(lam^2-C1) + B2*lam^2/(lam^2-C2) + B3* lam^2/(lam^2-C3)
    const Float n_squared = 1.0f + (B1 * lambda2) / (lambda2 - C1) + (B2 * lambda2) / (lambda2 - C2) + (B3 * lambda2) / (lambda2 - C3);
    
    return std::sqrt(n_squared);
}

std::vector<ReflectionPair> RealisticCamera::GenerateReflectionEvents() const {
    const int totalSequenceCount =
        elementInterfaces.size() * (elementInterfaces.size() - 2) / 2;

    std::vector<ReflectionPair> events;
    events.reserve(totalSequenceCount);

    for (int reflectTo = 0; reflectTo < elementInterfaces.size(); reflectTo++) {
        for (int reflectFrom = elementInterfaces.size() - 1; reflectFrom > reflectTo; reflectFrom--) {

            const bool involvesAperture =
                (elementInterfaces[reflectTo].curvatureRadiusX == 0) ||
                (elementInterfaces[reflectFrom].curvatureRadiusX == 0);

            if (involvesAperture)
                continue;

            // Ray goes forward to reflectFrom, then back to reflectTo
            events.emplace_back(reflectFrom, reflectTo);
        }
    }

    return events;
}

bool RealisticCamera::AdvanceRayThroughElements(
    Ray &ray,
    const int elementIndex,
    const Float frontZ,
    const LensElementInterface *elements,
    Float wavelengthNm) const
{
    const LensElementInterface &element = elements[elementIndex];

    // Compute element Z position
    Float elementZ = frontZ;
    for (int j = 0; j < elementIndex; j++)
        elementZ += elements[j].thickness;

    Float t;
    Normal3f n;
    const bool isStop = (element.curvatureRadiusX == 0);

    if (isStop) {
        t = (elementZ - ray.o.z) / ray.d.z;
        if (t < 0) return false;
    } else {
        Float zCenter = elementZ + element.curvatureRadiusX;
        // Use unified intersection function
        if (!IntersectLensElement(element.curvatureRadiusX, element.curvatureRadiusY, 
                                  zCenter, ray, &t, &n))
            return false;
    }

    const Point3f pHit = ray(t);
    if (Sqr(pHit.x) + Sqr(pHit.y) > Sqr(element.apertureRadius))
        return false;

    ray.o = pHit;

    if (!isStop) {
        // Get refractive indices (either wavelength-dependent or simple eta)
        Float eta_i = 1.0f; // Air by default
        Float eta_t;
        
        if (sellmeier) {
            // Full chromatic dispersion using Sellmeier
            if (elementIndex > 0) {
                const LensElementInterface& prevElement = elements[elementIndex - 1];
                eta_i = Sellmeier(wavelengthNm, prevElement.B1, prevElement.B2, prevElement.B3,
                                prevElement.C1, prevElement.C2, prevElement.C3);
            }
            eta_t = Sellmeier(wavelengthNm, element.B1, element.B2, element.B3,
                            element.C1, element.C2, element.C3);
        } else {
            // Use simple eta values (monochromatic)
            if (elementIndex > 0 && elements[elementIndex - 1].eta != 1.0f) {
                eta_i = elements[elementIndex - 1].eta;
            }
            eta_t = element.eta;
        }
        
        Vector3f wt;
        if (!Refract(Normalize(-ray.d), n, eta_t / eta_i, nullptr, &wt))
            return false;

        ray.d = wt;
    }

    return true;
}

// Perfect mirror reflection at a lens surface + Fresnel attenuation
Float RealisticCamera::ReflectAtElement(
    Ray& ray,
    int elementIndex,
    Float frontZ,
    Float wavelengthNm) const
{
    const LensElementInterface &element = elementInterfaces[elementIndex];

    Float reflectZ = frontZ;
    for (int j = 0; j < elementIndex; j++)
        reflectZ += elementInterfaces[j].thickness;

    const Float zCenter = reflectZ + element.curvatureRadiusX;

    Float t;
    Normal3f n;
    // Use unified intersection function
    if (!IntersectLensElement(element.curvatureRadiusX, element.curvatureRadiusY, zCenter, ray, &t, &n))
        return 0.0f;

    const Point3f pReflect = ray(t);
    if (Sqr(pReflect.x) + Sqr(pReflect.y) > Sqr(element.apertureRadius))
        return 0.0f;

    ray.o = pReflect;

    const Vector3f incident = Normalize(ray.d);
    ray.d = Normalize(incident - 2 * Dot(incident, n) * Vector3f(n));

    // Calculate Fresnel reflectance for this wavelength
    const Float cosTheta = AbsDot(ray.d, n);
    
    Float eta;
    if (sellmeier) {
        eta = Sellmeier(wavelengthNm, element.B1, element.B2, element.B3,
                       element.C1, element.C2, element.C3);
    } else {
        eta = element.eta;
    }

    return FrDielectric(cosTheta, eta);
}

// Trace a single flare ray with chromatic dispersion
// Helper function to trace a ray through the lens system with reflections
bool RealisticCamera::TraceFlareRayPath(
    Ray& ray,
    const ReflectionPair& event,
    Float frontZ,
    Float wavelengthNm,
    Float& intensityScale) const
{
    // To first reflection
    for (int i = 0; i < event.reflectFrom; i++) {
        if (!AdvanceRayThroughElements(ray, i, frontZ, elementInterfaces.data(), wavelengthNm))
            return false;
    }

    Float reflectance = ReflectAtElement(ray, event.reflectFrom, frontZ, wavelengthNm);
    if (reflectance == 0.0f) return false;
    intensityScale *= reflectance;

    // Between reflections
    for (int i = event.reflectFrom + 1; i < event.reflectTo; i++) {
        if (!AdvanceRayThroughElements(ray, i, frontZ, elementInterfaces.data(), wavelengthNm))
            return false;
    }

    reflectance = ReflectAtElement(ray, event.reflectTo, frontZ, wavelengthNm);
    if (reflectance == 0.0f) return false;
    intensityScale *= reflectance;

    // To film
    for (int i = event.reflectTo + 1; i < elementInterfaces.size(); i++) {
        if (!AdvanceRayThroughElements(ray, i, frontZ, elementInterfaces.data(), wavelengthNm))
            return false;
    }

    return true;
}

// Trace a single flare ray with chromatic dispersion
void RealisticCamera::TraceFlareRay(
    const Light& light,
    const std::vector<ReflectionPair>& reflectionEvents,
    int lightSampleIndex,
    int sampleIndex)
{
    // Sample light emission
    const Point2f u1(RadicalInverse(0, lightSampleIndex + sampleIndex),
               RadicalInverse(1, lightSampleIndex + sampleIndex));
    const Point2f u2(RadicalInverse(2, lightSampleIndex + sampleIndex),
               RadicalInverse(3, lightSampleIndex + sampleIndex));

    const Float wavelengthSample = RadicalInverse(6, lightSampleIndex + sampleIndex);
    const Float reflectionEventSample = RadicalInverse(7, lightSampleIndex + sampleIndex);
    SampledWavelengths lambda = SampledWavelengths::SampleVisible(wavelengthSample);
    auto leSample = light.SampleLe(u1, u2, lambda, 0);
    if (!leSample) return;

    SampledSpectrum baseIntensity = leSample->L;
    if (!baseIntensity || leSample->pdfPos == 0 || leSample->pdfDir == 0) return;

    // Choose reflection event
    const int eventIndex =
        std::min(static_cast<int>(reflectionEventSample * reflectionEvents.size()),
                 static_cast<int>(reflectionEvents.size() - 1));

    const ReflectionPair &event = reflectionEvents[eventIndex];

    // Sample point on first lens surface
    const Point2f disk = SampleUniformDiskConcentric(
        Point2f(RadicalInverse(4, lightSampleIndex + sampleIndex),
                RadicalInverse(5, lightSampleIndex + sampleIndex)));

    const Point2f pLens2d = elementInterfaces[0].apertureRadius * disk;
    const Float frontZ = -LensFrontZ();

    Point3f pLens(pLens2d.x, pLens2d.y, frontZ + elementInterfaces[0].curvatureRadiusX);

    const Transform cameraFromLens = Scale(1, 1, -1);
    pLens = cameraFromLens(pLens);

    Ray rayBase(leSample->ray.o, Normalize(pLens - leSample->ray.o));
    rayBase = Ray(Point3f(rayBase.o.x, rayBase.o.y, -rayBase.o.z),
                  Vector3f(rayBase.d.x, rayBase.d.y, -rayBase.d.z));

    if (sellmeier) {
        // SELLMEIER MODE: Trace each wavelength separately for chromatic dispersion
        for (int waveIdx = 0; waveIdx < NSpectrumSamples; waveIdx++) {
            const Float wavelengthNm = lambda[waveIdx];
            Ray ray = rayBase;
            Float intensityScale = 1.0f;

            if (!TraceFlareRayPath(ray, event, frontZ, wavelengthNm, intensityScale))
                continue;

            // Hit film plane
            if (std::abs(ray.d.z) < 1e-6f) continue;

            const Float tFilm = -ray.o.z / ray.d.z;
            Point3f pFilm = ray(tFilm);
            pFilm = Point3f(pFilm.x, pFilm.y, -pFilm.z);

            if (!Inside(Point2f(pFilm.x, pFilm.y), physicalExtent))
                continue;

            const Vector2f raster = physicalExtent.Offset(Point2f(pFilm.x, pFilm.y));

            const Point2i pixel(
                Clamp(static_cast<int>(raster.x * film.FullResolution().x), 0,
                      film.FullResolution().x - 1),
                Clamp(static_cast<int>(raster.y * film.FullResolution().y), 0,
                      film.FullResolution().y - 1));

            // Apply attenuation to only THIS wavelength's intensity
            SampledSpectrum contribution(0.0f);
            contribution[waveIdx] = baseIntensity[waveIdx] * intensityScale;

            // Normalize by total number of samples
            const SampledSpectrum normalizedContribution = contribution / 
                static_cast<Float>(flareSamples);

            film.AddSplat(Point2f(pixel.x + 0.5f, pixel.y + 0.5f), normalizedContribution, lambda);
        }
    } else {
        // ETA MODE: Single ray trace (no chromatic dispersion)
        const Float wavelengthNm = lambda[NSpectrumSamples / 2];
        Ray ray = rayBase;
        Float intensityScale = 1.0f;

        if (!TraceFlareRayPath(ray, event, frontZ, wavelengthNm, intensityScale))
            return;

        // Hit film plane
        if (std::abs(ray.d.z) < 1e-6f) return;

        const Float tFilm = -ray.o.z / ray.d.z;
        Point3f pFilm = ray(tFilm);
        pFilm = Point3f(pFilm.x, pFilm.y, -pFilm.z);

        if (!Inside(Point2f(pFilm.x, pFilm.y), physicalExtent))
            return;

        const Vector2f raster = physicalExtent.Offset(Point2f(pFilm.x, pFilm.y));

        const Point2i pixel(
            Clamp(static_cast<int>(raster.x * film.FullResolution().x), 0,
                  film.FullResolution().x - 1),
            Clamp(static_cast<int>(raster.y * film.FullResolution().y), 0,
                  film.FullResolution().y - 1));

        // Apply uniform attenuation to ALL wavelengths
        const SampledSpectrum attenuatedIntensity = baseIntensity * intensityScale;

        // Normalize by total number of samples
        const SampledSpectrum normalizedIntensity = attenuatedIntensity / static_cast<Float>(flareSamples);

        film.AddSplat(Point2f(pixel.x + 0.5f, pixel.y + 0.5f), normalizedIntensity, lambda);
    }
}

void RealisticCamera::RenderLensFlare(const std::vector<Light> &lights) {
    LOG_VERBOSE("Starting lens flare rendering with %d lights", lights.size());
    if (elementInterfaces.size() < 2) {
        LOG_VERBOSE("Not enough lens elements for flare: %d", elementInterfaces.size());
        return;
    }

    const std::vector<ReflectionPair> reflectionEvents = GenerateReflectionEvents();
    LOG_VERBOSE("Created %d reflection sequences", reflectionEvents.size());
    if (reflectionEvents.empty()) {
        LOG_VERBOSE("No internal reflections possible");
        return;
    }
    
    LOG_VERBOSE("Processing %d lights with %d total flare samples", lights.size(), flareSamples);

    ParallelFor(0, flareSamples, [&](const int globalSampleIndex) {
        const int lightIndex = globalSampleIndex % lights.size();
        const int localSampleIndex = globalSampleIndex / lights.size();
        const Light light = lights[lightIndex];

        TraceFlareRay(light, reflectionEvents, lightIndex, localSampleIndex);
    });
    LOG_VERBOSE("Finished lens flare rendering");
}

}