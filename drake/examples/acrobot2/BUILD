# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
    "drake_cc_binary",
)
load("//tools:install_data.bzl", "install_data")
load("//tools:lint.bzl", "add_lint_tests")

drake_cc_library(
    name = "acrobot_state_vector",
    srcs = ["gen/acrobot_state_vector.cc"],
    hdrs = ["gen/acrobot_state_vector.h"],
    deps = [
        "//drake/systems/framework:vector",
    ],
)

drake_cc_library(
    name = "acrobot_lcm",
    srcs = ["acrobot_lcm.cc"],
    hdrs = ["acrobot_lcm.h"],
    deps = [
        ":acrobot_state_vector",
        "//drake/lcmtypes:acrobot",
        "//drake/systems/framework:leaf_system",
        "//drake/systems/framework:vector",
    ],
)

drake_cc_library(
    name = "acrobot_plant",
    srcs = ["acrobot_plant.cc"],
    hdrs = ["acrobot_plant.h"],
    deps = [
        ":acrobot_state_vector",
        "//drake/multibody:rigid_body_tree",
        "//drake/systems/controllers:linear_quadratic_regulator",
        "//drake/systems/framework",
        "//drake/systems/sensors:rotary_encoders",
    ],
)

drake_cc_binary(
    name = "acrobot_run_passive",
    srcs = ["acrobot_run_passive.cc"],
    add_test_rule = 1,
    data = [":models"],
    test_rule_args = ["-realtime_factor=0.0"],
    deps = [
        ":acrobot_plant",
        "//drake/common:find_resource",
        "//drake/lcm",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/joints",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis:simulator",
        "//drake/systems/framework:diagram",
        "@com_github_gflags_gflags//:gflags",
    ],
)

drake_cc_googletest(
    name = "acrobot_urdf_dynamics_test",
    data = [":models"],
    deps = [
        ":acrobot_plant",
        "//drake/common:eigen_matrix_compare",
        "//drake/common:find_resource",
        "//drake/multibody/joints",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant",
    ],
)

install_data()

add_lint_tests()
