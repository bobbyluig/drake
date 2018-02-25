# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
    "drake_cc_binary",
)
load("//tools/install:install_data.bzl", "install_data")
load("//tools/lint:lint.bzl", "add_lint_tests")

drake_cc_library(
    name = "rod_element",
    srcs = [
        "rod_element.cc",
    ],
    hdrs = [
        "rod_element.h",
    ],
    deps = [
        "//drake/multibody/multibody_tree",
    ],
)

drake_cc_library(
    name = "cosserat_rod_plant",
    srcs = [
        "cosserat_rod_plant.cc",
    ],
    hdrs = [
        "cosserat_rod_plant.h",
    ],
    deps = [
        ":rod_element",
        "//drake/lcm",
        "//drake/lcmtypes:viewer",
        "//drake/multibody/multibody_tree",
        "//drake/systems/framework",
        "//drake/systems/rendering:drake_visualizer_client",
        "//drake/systems/rendering:pose_bundle",
    ],
)

install_data()

add_lint_tests()
