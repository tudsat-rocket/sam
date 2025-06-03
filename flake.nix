{
  description = "sam";
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    rust-overlay = {
      url = "github:oxalica/rust-overlay";
      inputs = {
        nixpkgs.follows = "nixpkgs";
      };
    };
  };
  outputs =
    {
      rust-overlay,
      nixpkgs,
      flake-utils,
      ...
    }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = nixpkgs.legacyPackages.${system}.extend (import rust-overlay);
        rustToolchain = pkgs.rust-bin.fromRustupToolchainFile ./rust-toolchain.toml;

        nativeBuildInputs = [
          pkgs.pkg-config
          pkgs.wrapGAppsHook
        ];
        buildInputs = with pkgs; [
          rustToolchain
          glib
          libudev-zero
          gtk3
          atkmm
        ];
        LD_LIBRARY_PATH =
          with pkgs;
          nixpkgs.lib.makeLibraryPath [
            libGL
            libxkbcommon
          ];

        sam = pkgs.rustPlatform.buildRustPackage {
          pname = "sam";
          version = "0.1.0";
          src = ./.;

          cargoLock = {
            lockFile = ./Cargo.lock;
            allowBuiltinFetchGit = true;
          };
          inherit nativeBuildInputs buildInputs LD_LIBRARY_PATH;

        };
      in
      {
        devShells.default = pkgs.mkShell {
          inherit nativeBuildInputs buildInputs LD_LIBRARY_PATH;
        };

        formatter = pkgs.nixfmt-rfc-style;

        packages = {
          default = sam;
        };
      }
    );
}
