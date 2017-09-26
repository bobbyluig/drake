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
    name = "rgbd_to_lcm_point_cloud",
    srcs = ["rgbd_to_lcm_point_cloud.cc"],
    hdrs = ["rgbd_to_lcm_point_cloud.h"],
    deps = [
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
        "//drake/systems/sensors:rgbd_camera",
        "@com_github_gflags_gflags//:gflags",
    ],
)

install_data()
