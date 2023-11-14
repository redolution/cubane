{
  description = "IPL replacement modchip for the Nintendo GameCube";

  inputs = {
    flake-compat = {
      url = "github:edolstra/flake-compat";
      flake = false;
    };
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-parts = {
      url = "github:hercules-ci/flake-parts";
      inputs.nixpkgs-lib.follows = "nixpkgs";
    };
    systems.flake = false;
    rust-overlay.url = "github:oxalica/rust-overlay";
  };

  outputs = { ... } @ inputs: inputs.flake-parts.lib.mkFlake {
    inherit inputs;
  } ({ config, flake-parts-lib, getSystem, inputs, lib, options, ... }:
    let
      rootConfig = config;
      rootOptions = options;
    in
    {
      _file = ./flake.nix;
      imports = [ ];
      config.perSystem = { config, inputs', nixpkgs, options, pkgs, system, ... }:
        let
          systemConfig = config;
          systemOptions = options;
        in
        {
          _file = ./flake.nix;
          config = {
            _module.args.pkgs = import inputs.nixpkgs {
              inherit system;
              overlays = [
                (import inputs.rust-overlay)
              ];
              config = { };
            };

            devShells.default = pkgs.callPackage
              ({ mkShell
              , rust-bin
              , flip-link
              , probe-rs
              , elf2uf2-rs
              }: mkShell {
                name = "cubane";
                nativeBuildInputs = [
                  # RTIC 2.0 currently requires nightly
                  (rust-bin.nightly.latest.default.override {
                    extensions = [
                      "rust-analyzer"
                      "rust-src"
                    ];
                    targets = [ "thumbv6m-none-eabi" ];
                  })
                  flip-link
                  probe-rs
                  elf2uf2-rs
                ];
              })
              { };
          };
        };
      config.systems = import inputs.systems;
  });
}
