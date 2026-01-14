{
  description = "Swarm Robot ESP32 Development Environment";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
  };

  outputs = { self, nixpkgs }:
    let
      system = "x86_64-linux";
      pkgs = import nixpkgs {
        inherit system;
        config.allowUnfree = true;
      };
    in
    {
      devShells.${system}.default = pkgs.mkShell {
        name = "esp32-swarm-env";

        buildInputs = with pkgs; [
          esptool
          esp-idf
          minicom
          usbutils
        ];
      };
    };
}
