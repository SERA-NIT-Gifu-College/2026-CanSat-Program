{
  description = "A very basic flake";

  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs?ref=nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = inputs@{self, nixpkgs, flake-utils}:
    inputs.flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; config = {}; };
        my-pico-sdk = pkgs.pico-sdk.override {
          withSubmodules = true;
        };
      in
      {
        devShells.default = pkgs.mkShell {
          name = "pico env";
          packages = with pkgs; [
            cmake
            doxygen
            gcc-arm-embedded
            openocd-rp2040
            picotool
            pioasm
            pkg-config
            python3
            tio
          ] ++ [ my-pico-sdk ];
          shellHook = ''
            export GCC_ARM_PATH="${pkgs.gcc-arm-embedded}"
            export PICO_SDK_PATH="${my-pico-sdk}/lib/pico-sdk"
          '';
        };
      }
    );
}
