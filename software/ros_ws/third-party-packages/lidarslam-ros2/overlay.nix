final: prev:
{
  graph-based-slam = final.callPackage ./graph_based_slam.nix {};
  lidarslam = final.callPackage ./lidarslam.nix {};
  lidarslam-msgs = final.callPackage ./lidarslam_msgs.nix {};
  scanmatcher = final.callPackage ./scanmatcher.nix {};
}
