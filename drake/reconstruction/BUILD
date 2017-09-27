# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load(
    "//tools:drake.bzl",
    "drake_cc_library",
    "drake_cc_binary",
)
load("//tools:install_data.bzl", "install_data")
load("//tools:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "rgbd_camera_2",
    srcs = ["rgbd_camera_2.cc"],
    hdrs = ["rgbd_camera_2.h"],
    deps = [
        "//drake/math:geometric_transform",
        "//drake/multibody:rigid_body_frame",
        "//drake/multibody:rigid_body_tree",
        "//drake/systems/framework",
        "//drake/systems/framework:diagram_builder",
        "//drake/systems/primitives",
        "//drake/systems/rendering:pose_vector",
        "//drake/systems/sensors:camera_info",
        "//drake/systems/sensors:image",
        "//drake/systems/sensors:vtk_util",
        "@eigen",
        "@vtk//:vtkCommonCore",
        "@vtk//:vtkCommonDataModel",
        "@vtk//:vtkCommonTransforms",
        "@vtk//:vtkFiltersGeneral",
        "@vtk//:vtkFiltersSources",
        "@vtk//:vtkIOGeometry",
        "@vtk//:vtkIOImage",
        "@vtk//:vtkRenderingCore",
        "@vtk//:vtkRenderingOpenGL2",
    ],
)

drake_cc_library(
    name = "rgbd_to_lcm_point_cloud",
    srcs = ["rgbd_to_lcm_point_cloud.cc"],
    hdrs = ["rgbd_to_lcm_point_cloud.h"],
    deps = [
        ":rgbd_camera_2",
        "//drake/systems/sensors:rgbd_camera",
        "@eigen",
        "@lcmtypes_bot2_core",
        "@lcmtypes_robotlocomotion",
        "@zlib",
    ],
)

drake_cc_binary(
    name = "reconstruct",
    srcs = ["reconstruct.cc"],
    data = [":models"],
    deps = [
        ":rgbd_camera_2",
        ":rgbd_to_lcm_point_cloud",
        "//drake/common:find_resource",
        "//drake/lcm",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis",
        "//drake/systems/lcm",
        "//drake/systems/sensors:image_to_lcm_image_array_t",
        "@com_github_gflags_gflags//:gflags",
    ],
)

install_data()
