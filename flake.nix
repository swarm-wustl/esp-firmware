{
  description = "Swarm ESP32 Firmware";
  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/develop";
    esp-dev.url = "github:mirrexagon/nixpkgs-esp-dev";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";
  };
  outputs = { self, nix-ros-overlay, nixpkgs, esp-dev }:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ 
            nix-ros-overlay.overlays.default
            # This overlay overrides the specific python package causing the failure
            (final: prev: {
              pythonPackagesExtensions = prev.pythonPackagesExtensions ++ [
                (python-final: python-prev: {
                  colcon-ros = python-prev.colcon-ros.overridePythonAttrs (old: {
                    # Ignore the setuptools version conflict between catkin-pkg and colcon-core
                    catchConflicts = false;
                  });
                })
              ];
            })
          ];
        };
        rosDistro = "humble";

        esp-shell = esp-dev.devShells.${system}.esp32-idf;
      in {
        devShells.default = pkgs.mkShell {
          inputsFrom = [ esp-shell ];

          name = "Swarm";
          packages = with pkgs; [
            colcon
            python3
            python3Packages.empy
            python3Packages.lark

            (with rosPackages.${rosDistro}; buildEnv {
              paths = [
                ros-core
                ament-cmake
                ament-cmake-core
                ament-cmake-python
                python-cmake-module
                teleop-twist-keyboard

                geometry-msgs
                std-msgs

                rosidl-typesupport-cpp
                rosidl-typesupport-introspection-cpp
              ];
            })
          ];
        };
      });
}
