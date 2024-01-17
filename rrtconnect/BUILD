package(default_visibility = ["//visibility:public"])

licenses(["notice"])

exports_files(["LICENSE"])

cc_library(
    name = "binary_search",
    hdrs = [
        "binary_search.h",
    ],
    deps = [
        "@x_edr_eigenmath//eigenmath",
    ],
)

cc_library(
    name = "path_metric",
    srcs = [
        "path_metric.cc",
    ],
    hdrs = [
        "path_metric.h",
    ],
    deps = [
        ":binary_search",
        ":rrt",
        "@com_google_absl//absl/log",
    ],
)

cc_library(
    name = "rrt",
    srcs = [
        "rrt.cc",
        "rrt_connect.cc",
    ],
    hdrs = [
        "rrt.h",
        "rrt_connect.h",
    ],
    deps = [
        "@boost.graph",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/container:node_hash_set",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/time",
        "@com_google_absl//absl/types:optional",
        "@x_edr_eigenmath//eigenmath",
        "@x_edr_eigenmath//eigenmath:quasi_random_vector",
    ],
)

cc_test(
    name = "rrt_connect_test",
    srcs = [
        "rrt_connect_test.cc",
    ],
    deps = [
        ":path_metric",
        ":rrt",
        "@com_google_googletest//:gtest_main",
        "@x_edr_eigenmath//eigenmath",
    ],
)

cc_test(
    name = "binary_search_test",
    srcs = ["binary_search_test.cc"],
    deps = [
        ":binary_search",
        "@com_google_googletest//:gtest_main",
        "@x_edr_eigenmath//eigenmath",
    ],
)
