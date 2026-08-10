final: prev: {
  cvp-mesh-planner = final.callPackage ./cvp-mesh-planner.nix { };
  dijkstra-mesh-planner = final.callPackage ./dijkstra-mesh-planner.nix { };
  mbf-mesh-core = final.callPackage ./mbf-mesh-core.nix { };
  mbf-mesh-nav = final.callPackage ./mbf-mesh-nav.nix { };
  mesh-controller = final.callPackage ./mesh-controller.nix { };
  mesh-layers = final.callPackage ./mesh-layers.nix { };
  mesh-map = final.callPackage ./mesh-map.nix { };
  mesh-navigation = final.callPackage ./mesh-navigation.nix { };
}
