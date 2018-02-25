load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
    "drake_cc_binary",
)
load("//tools/install:install_data.bzl", "install_data")
load("//tools/lint:lint.bzl", "add_lint_tests")

drake_cc_library(
    name = "poking_element",
    srcs = [
        "poking_element.cc",
    ],
    hdrs = [
        "poking_element.h",
    ],
    deps = [
        "//drake/multibody/multibody_tree",
    ],
)
