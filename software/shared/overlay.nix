final: prev: {
  crc = final.callPackage ./crc { };
  fd-wrapper = final.callPackage ./fd-wrapper { };
  hi-can = final.callPackage ./hi-can { };
  hi-can-raw = final.callPackage ./hi-can-raw { };
  hi-can-net = final.callPackage ./hi-can-net { };
  ptr-wrapper = final.callPackage ./ptr-wrapper { };
  simple-networking = final.callPackage ./simple-networking { };
  type-demangle = final.callPackage ./type-demangle { };
}
