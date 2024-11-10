final: prev: {
  fd-wrapper = final.callPackage ./fd-wrapper { };
  hi-can-lib = final.callPackage ./hi-can-lib { };
  hi-can-raw = final.callPackage ./hi-can-raw { };
  hi-can-net = final.callPackage ./hi-can-net { };
}
