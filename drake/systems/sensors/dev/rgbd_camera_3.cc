#include "rgbd_camera_3.h"

#include <array>
#include <fstream>
#include <limits>
#include <map>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkCubeSource.h>
#include <vtkCylinderSource.h>
#include <vtkImageExport.h>
#include <vtkNew.h>
#include <vtkOBJReader.h>
#include <vtkPNGReader.h>
#include <vtkPlaneSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkVersion.h>
#include <vtkWindowToImageFilter.h>

#if VTK_MAJOR_VERSION >= 6
#include <vtkAutoInit.h>
#endif

#include "drake/math/roll_pitch_yaw.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/zero_order_hold.h"
#include "drake/systems/rendering/pose_vector.h"
#include "drake/systems/sensors/camera_info.h"
#include "drake/systems/sensors/image.h"
#include "drake/systems/sensors/vtk_util.h"

#if VTK_MAJOR_VERSION >= 6
VTK_AUTOINIT_DECLARE(vtkRenderingOpenGL2)
#endif

// TODO(kunimatsu-tri) Refactor RenderingWorld out from RgbdCamera3,
// so that other vtk dependent sensor simulators can share the RenderingWorld
// without duplicating it.

namespace drake {
namespace systems {
namespace sensors {

using vtk_util::ConvertToVtkTransform;
using vtk_util::MakeVtkPointerArray;
using geometry::GeometryState;
using geometry::GeometryIndex;
using geometry::ShapeReifier;
using geometry::Shape;
using geometry::internal::InternalGeometry;

namespace {

const int kPortStateInput = 0;

const double kClippingPlaneNear = 0.01;
const double kClippingPlaneFar = 100.;

const int kImageWidth = 640;  // In pixels
const int kImageHeight = 480;  // In pixels

// For Zbuffer value conversion.
const double kA = kClippingPlaneFar / (kClippingPlaneFar - kClippingPlaneNear);
const double kB = -kA * kClippingPlaneNear;

std::string RemoveFileExtension(const std::string& filepath) {
  const size_t last_dot = filepath.find_last_of(".");
  if (last_dot == std::string::npos) {
    DRAKE_DEMAND(false);
  }
  return filepath.substr(0, last_dot);
}

// Register the object factories for the vtkRenderingOpenGL2 module.
struct ModuleInitVtkRenderingOpenGL2 {
  ModuleInitVtkRenderingOpenGL2() {
#if VTK_MAJOR_VERSION >= 6
    VTK_AUTOINIT_CONSTRUCT(vtkRenderingOpenGL2)
#endif
  }
};

// Updates VTK rendering related objects including vtkRenderWindow,
// vtkWindowToImageFilter and vtkImageExporter, so that VTK reflects
// vtkActors' pose update for rendering.
void PerformVTKUpdate(const vtkNew<vtkRenderWindow>& window,
                      const vtkNew<vtkWindowToImageFilter>& filter,
                      const vtkNew<vtkImageExport>& exporter) {
  window->Render();
  filter->Modified();
  filter->Update();
  exporter->Update();
}

}  // namespace

void RgbdCamera3::DepthImageToPointCloud(const ImageDepth32F& depth_image,
                                         const CameraInfo& camera_info,
                                         Eigen::Matrix3Xf* point_cloud) {
  if (depth_image.size() != point_cloud->cols()) {
    point_cloud->resize(3, depth_image.size());
  }

  const int height = depth_image.height();
  const int width = depth_image.width();
  const float cx = static_cast<const float>(camera_info.center_x());
  const float cy = static_cast<const float>(camera_info.center_y());
  const float fx_inv = static_cast<const float>(1.f / camera_info.focal_x());
  const float fy_inv = static_cast<const float>(1.f / camera_info.focal_y());

  Eigen::Matrix3Xf& pc = *point_cloud;
  for (int v = 0; v < height; ++v) {
    for (int u = 0; u < width; ++u) {
      float z = depth_image.at(u, v)[0];
      if (z == InvalidDepth::kTooClose || z == InvalidDepth::kTooFar) {
        pc(0, v * width + u) = InvalidDepth::kTooFar;
        pc(1, v * width + u) = InvalidDepth::kTooFar;
        pc(2, v * width + u) = InvalidDepth::kTooFar;
      } else {
        pc(0, v * width + u) = z * (u - cx) * fx_inv;
        pc(1, v * width + u) = z * (v - cy) * fy_inv;
        pc(2, v * width + u) = z;
      }
    }
  }
}

class RgbdCamera3::ShapeToVtk : public ShapeReifier {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ShapeToVtk)

  ShapeToVtk() = default;
  ~ShapeToVtk() override = default;

  void Reify(const Shape& shape, vtkNew<vtkActor>& actor) {
    shape.Reify(this);

    actor->SetMapper(mapper_.GetPointer());
    actor->GetProperty()->SetColor(0.8, 0.8, 0.8);
  }

  void ImplementGeometry(const geometry::Sphere& sphere) override {
    vtkNew<vtkSphereSource> vtk_sphere;
    vtk_sphere->SetRadius(sphere.get_radius());
    vtk_sphere->SetThetaResolution(50);
    vtk_sphere->SetPhiResolution(50);
    mapper_->SetInputConnection(vtk_sphere->GetOutputPort());
  }

  void ImplementGeometry(const geometry::Cylinder& cylinder) override {
    vtkNew<vtkCylinderSource> vtk_cylinder;
    vtk_cylinder->SetHeight(cylinder.get_length());
    vtk_cylinder->SetRadius(cylinder.get_radius());
    vtk_cylinder->SetResolution(50);

    // Since the cylinder in vtkCylinderSource is y-axis aligned, we need
    // to rotate it to be z-axis aligned because that is what Drake uses.
    vtkNew<vtkTransform> transform;
    transform->RotateX(90);
    vtkNew<vtkTransformPolyDataFilter> transform_filter;
    transform_filter->SetInputConnection(vtk_cylinder->GetOutputPort());
    transform_filter->SetTransform(transform.GetPointer());
    transform_filter->Update();

    mapper_->SetInputConnection(transform_filter->GetOutputPort());
  }

  void ImplementGeometry(const geometry::HalfSpace& half_space) override {
    throw std::runtime_error("Not implemented yet.");
  }

 private:
  vtkNew<vtkPolyDataMapper> mapper_;
};

class RgbdCamera3::Impl : private ModuleInitVtkRenderingOpenGL2 {
 public:
  Impl(const GeometrySystem<double>& geometry,
       double depth_range_near, double depth_range_far,
       double fov_y, bool show_window);

  Impl(const GeometrySystem<double>& geometry,
       const Eigen::Vector3d& position, const Eigen::Vector3d& orientation,
       double depth_range_near, double depth_range_far,
       double fov_y, bool show_window);

  ~Impl() {}

  float CheckRangeAndConvertToMeters(float z_buffer_value) const;

  const Eigen::Isometry3d& color_camera_optical_pose() const {
    return X_BC_;
  }

  const Eigen::Isometry3d& depth_camera_optical_pose() const {
    return X_BD_;
  }

  const CameraInfo& color_camera_info() const { return color_camera_info_; }

  const CameraInfo& depth_camera_info() const { return depth_camera_info_; }

  const GeometrySystem<double>& geometry() const { return geometry_; }

  // These are the calculator method implementations for the four output ports.
  void OutputColorImage(ImageRgba8U* color_image) const;
  void OutputDepthImage(ImageDepth32F* depth_image) const;
  void OutputPoseVector(rendering::PoseVector<double>* pose_vector) const;

 private:
  void CreateRenderingWorld();

  // TODO(sherm1) This should be the calculator for a cache entry containing
  // the VTK update that must be valid before outputting any image info. For
  // now it has to be repeated before each image output port calculation.
  void UpdateModelPoses() const;

  // Initializes camera pose first and sets camera pose in the world frame X_WC.
  void SetModelTransformMatrixToVtkCamera(
      vtkCamera* camera, const vtkSmartPointer<vtkTransform>& X_WC) const;

  const GeometrySystem<double>& geometry_;
  const CameraInfo color_camera_info_{};
  const CameraInfo depth_camera_info_{};
  const Eigen::Isometry3d X_BC_;
  const Eigen::Isometry3d X_BD_;
  const Eigen::Isometry3d X_WB_initial_;
  const double depth_range_near_{};
  const double depth_range_far_{};

  std::map<GeometryIndex, vtkSmartPointer<vtkActor>> id_object_map_;
  vtkNew<vtkRenderer> color_depth_renderer_;
  vtkNew<vtkRenderWindow> color_depth_render_window_;
  vtkNew<vtkWindowToImageFilter> color_filter_;
  vtkNew<vtkWindowToImageFilter> depth_filter_;
  vtkNew<vtkImageExport> color_exporter_;
  vtkNew<vtkImageExport> depth_exporter_;
};

RgbdCamera3::Impl::Impl(const GeometrySystem<double>& geometry,
                        const Eigen::Vector3d& position,
                        const Eigen::Vector3d& orientation,
                        double depth_range_near, double depth_range_far,
                        double fov_y, bool show_window)
    : geometry_(geometry),
      color_camera_info_(kImageWidth, kImageHeight, fov_y),
      depth_camera_info_(kImageWidth, kImageHeight, fov_y),
      X_BC_(Eigen::Translation3d(0., 0.02, 0.) *
          (Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX()) *
              Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()))),
      X_BD_(Eigen::Translation3d(0., 0.02, 0.) *
          (Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX()) *
              Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()))),
      X_WB_initial_(
          Eigen::Translation3d(position[0], position[1], position[2]) *
              Eigen::Isometry3d(math::rpy2rotmat(orientation))),
      depth_range_near_(depth_range_near), depth_range_far_(depth_range_far) {
  DRAKE_DEMAND(depth_range_near_ >= kClippingPlaneNear);
  DRAKE_DEMAND(depth_range_far_ <= kClippingPlaneFar);
  DRAKE_DEMAND(depth_range_far_ - depth_range_near_ > 0);

  if (!show_window) {
    color_depth_render_window_->SetOffScreenRendering(1);
  }

  const vtkSmartPointer<vtkTransform> vtk_X_WC =
      ConvertToVtkTransform(X_WB_initial_ * X_BC_);

  color_depth_renderer_->SetBackground(0.8, 0.9, 1.0);
  auto camera = color_depth_renderer_->GetActiveCamera();
  camera->SetViewAngle(fov_y * 180. / M_PI);
  camera->SetClippingRange(kClippingPlaneNear, kClippingPlaneFar);
  SetModelTransformMatrixToVtkCamera(camera, vtk_X_WC);

  CreateRenderingWorld();

#if ((VTK_MAJOR_VERSION == 7) && (VTK_MINOR_VERSION >= 1)) || \
    (VTK_MAJOR_VERSION >= 8)
  color_depth_renderer_->SetUseDepthPeeling(1);
  color_depth_renderer_->UseFXAAOn();
#endif

  color_depth_render_window_->SetSize(color_camera_info_.width(),
                                      color_camera_info_.height());
  color_depth_render_window_->AddRenderer(color_depth_renderer_.GetPointer());

  color_filter_->SetInput(color_depth_render_window_.GetPointer());
  color_filter_->SetInputBufferTypeToRGBA();
  depth_filter_->SetInput(color_depth_render_window_.GetPointer());
  depth_filter_->SetInputBufferTypeToZBuffer();

  auto exporters = MakeVtkPointerArray(color_exporter_, depth_exporter_);
  auto filters = MakeVtkPointerArray(color_filter_, depth_filter_);

  for (int i = 0; i < 2; ++i) {
    filters[i]->SetMagnification(1);
    filters[i]->ReadFrontBufferOff();
    filters[i]->Update();
#if VTK_MAJOR_VERSION <= 5
    exporters[i]->SetInput(filters[i]->GetOutput());
#else
    exporters[i]->SetInputData(filters[i]->GetOutput());
#endif
    exporters[i]->ImageLowerLeftOff();
  }
}

RgbdCamera3::Impl::Impl(const GeometrySystem<double>& geometry,
                        double depth_range_near, double depth_range_far,
                        double fov_y, bool show_window)
    : Impl::Impl(geometry, Eigen::Vector3d(0., 0., 0.),
                 Eigen::Vector3d(0., 0., 0.),
                 depth_range_near, depth_range_far,
                 fov_y, show_window) {}

void RgbdCamera3::Impl::SetModelTransformMatrixToVtkCamera(
    vtkCamera* camera, const vtkSmartPointer<vtkTransform>& X_WC) const {
  // vtkCamera contains a transformation as the internal state and
  // ApplyTransform multiplies a given transformation on top of the internal
  // transformation. Thus, resetting 'Set{Position, FocalPoint, ViewUp}' is
  // needed here.
  camera->SetPosition(0., 0., 0.);
  camera->SetFocalPoint(0., 0., 1.);  // Sets z-forward.
  camera->SetViewUp(0., -1, 0.);  // Sets y-down. For the detail, please refere
  // to CameraInfo's document.
  camera->ApplyTransform(X_WC);
}

void RgbdCamera3::Impl::CreateRenderingWorld() {
  const GeometryState<double>& state = geometry_.get_initial_state();

  // TODO(bobbyluig): Support anchored geometries.
  for (const auto& pair : state.get_geometries()) {
    const InternalGeometry& geometry = pair.second;

    vtkNew<vtkActor> actor;
    ShapeToVtk reifier;

    reifier.Reify(geometry.get_shape(), actor);

    auto& X_WV = state.get_pose_in_world(geometry.get_id());
    vtkSmartPointer<vtkTransform> vtk_transform = ConvertToVtkTransform(X_WV);
    actor->SetUserTransform(vtk_transform);
    color_depth_renderer_->AddActor(actor.GetPointer());
    id_object_map_[geometry.get_engine_index()] = actor.GetPointer();
  }
}

void RgbdCamera3::Impl::OutputColorImage(ImageRgba8U* color_image) const {
  // TODO(sherm1) Should evaluate VTK cache entry.
  UpdateModelPoses();
  PerformVTKUpdate(color_depth_render_window_, color_filter_, color_exporter_);
  color_exporter_->Export(color_image->at(0, 0));
}

void RgbdCamera3::Impl::OutputDepthImage(ImageDepth32F* depth_image_out) const {
  // TODO(sherm1) Should evaluate VTK cache entry.
  UpdateModelPoses();
  PerformVTKUpdate(color_depth_render_window_, depth_filter_, depth_exporter_);
  depth_exporter_->Export(depth_image_out->at(0, 0));

  const int height = color_camera_info_.height();
  const int width = color_camera_info_.width();
  // TODO(kunimatsu-tri) Calculate this in a vertex shader.
  for (int v = 0; v < height; ++v) {
    for (int u = 0; u < width; ++u) {
      depth_image_out->at(u, v)[0] =
          CheckRangeAndConvertToMeters(depth_image_out->at(u, v)[0]);
    }
  }
}

void RgbdCamera3::Impl::OutputPoseVector(
    rendering::PoseVector<double>* camera_base_pose) const {
  Eigen::Translation<double, 3> trans =
      Eigen::Translation<double, 3>(X_WB_initial_.translation());
  camera_base_pose->set_translation(trans);
  Eigen::Quaterniond quat = Eigen::Quaterniond(X_WB_initial_.linear());
  camera_base_pose->set_rotation(quat);
}

void RgbdCamera3::Impl::UpdateModelPoses() const {
  const GeometryState<double>& state = geometry_.get_initial_state();

  // Updates body poses.
  for (const auto& pair : state.get_geometries()) {
    const InternalGeometry& geometry = pair.second;

    auto& X_WV = state.get_pose_in_world(geometry.get_id());
    vtkSmartPointer<vtkTransform> vtk_X_WV = ConvertToVtkTransform(X_WV);
    auto& actor = id_object_map_.at(geometry.get_engine_index());
    actor->SetUserTransform(vtk_X_WV);
  }
}

float RgbdCamera3::Impl::CheckRangeAndConvertToMeters(float z_buffer_value)
const {
  float checked_depth;
  // When the depth is either closer than kClippingPlaneNear or further than
  // kClippingPlaneFar, `z_buffer_value` becomes `1.f`.
  if (z_buffer_value == 1.f) {
    checked_depth = std::numeric_limits<float>::quiet_NaN();
  } else {
    auto depth = (float) (kB / (z_buffer_value - kA));

    if (depth > depth_range_far_) {
      checked_depth = InvalidDepth::kTooFar;
    } else if (depth < depth_range_near_) {
      checked_depth = InvalidDepth::kTooClose;
    } else {
      checked_depth = depth;
    }
  }

  return checked_depth;
}

RgbdCamera3::RgbdCamera3(const std::string& name,
                         const GeometrySystem<double>& geometry,
                         const Eigen::Vector3d& position,
                         const Eigen::Vector3d& orientation,
                         double depth_range_near,
                         double depth_range_far,
                         double fov_y,
                         bool show_window)
    : impl_(
    new RgbdCamera3::Impl(geometry,
                          position,
                          orientation,
                          depth_range_near,
                          depth_range_far,
                          fov_y,
                          show_window)) {
  Init(name);
}

void RgbdCamera3::Init(const std::string& name) {
  set_name(name);

  ImageRgba8U color_image(kImageWidth, kImageHeight);
  color_image_port_ = &this->DeclareAbstractOutputPort(
      sensors::ImageRgba8U(color_image), &RgbdCamera3::OutputColorImage);

  ImageDepth32F depth_image(kImageWidth, kImageHeight);
  depth_image_port_ = &this->DeclareAbstractOutputPort(
      sensors::ImageDepth32F(depth_image), &RgbdCamera3::OutputDepthImage);

  camera_base_pose_port_ = &this->DeclareVectorOutputPort(
      rendering::PoseVector<double>(), &RgbdCamera3::OutputPoseVector);
}

RgbdCamera3::~RgbdCamera3() = default;

const CameraInfo& RgbdCamera3::color_camera_info() const {
  return impl_->color_camera_info();
}

const CameraInfo& RgbdCamera3::depth_camera_info() const {
  return impl_->depth_camera_info();
}

const Eigen::Isometry3d& RgbdCamera3::color_camera_optical_pose() const {
  return impl_->color_camera_optical_pose();
}

const Eigen::Isometry3d& RgbdCamera3::depth_camera_optical_pose() const {
  return impl_->depth_camera_optical_pose();
}

const GeometrySystem<double>& RgbdCamera3::geometry() const {
  return impl_->geometry();
}

const OutputPort<double>&
RgbdCamera3::camera_base_pose_output_port() const {
  return *camera_base_pose_port_;
}

const OutputPort<double>&
RgbdCamera3::color_image_output_port() const {
  return *color_image_port_;
}

const OutputPort<double>&
RgbdCamera3::depth_image_output_port() const {
  return *depth_image_port_;
}

void RgbdCamera3::OutputPoseVector(
    const Context<double>& context,
    rendering::PoseVector<double>* pose_vector) const {
  impl_->OutputPoseVector(pose_vector);
}

void RgbdCamera3::OutputColorImage(const Context<double>& context,
                                   ImageRgba8U* color_image) const {
  impl_->OutputColorImage(color_image);
}

void RgbdCamera3::OutputDepthImage(const Context<double>& context,
                                   ImageDepth32F* depth_image) const {
  impl_->OutputDepthImage(depth_image);
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
