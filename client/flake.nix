{
  description = "dev shell for client software";

  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs?ref=nixos-unstable";
  };

  outputs = { self, nixpkgs }:
  let
    pkgs = import nixpkgs {
      system = "x86_64-linux";
    };
  in
  {
    devShells.x86_64-linux.default = pkgs.stdenv.mkDerivation {
      name = "shell";
      buildInputs = with pkgs; [
        boost
        raylib
        gcc
        gnumake
        pkg-config
        doxygen
        neovim
      ];
    };
  };
}
