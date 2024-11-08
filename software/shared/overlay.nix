final: prev: {
  hi-can-lib = prev.callPackage ../shared/hi-can-lib { };
  hi-can-raw = prev.callPackage ../shared/hi-can-raw { };
  hi-can-net = prev.callPackage ../shared/hi-can-net { };
}
