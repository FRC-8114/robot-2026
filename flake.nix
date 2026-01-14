{
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs";
    frc-nix = {
      url = "github:frc4451/frc-nix";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs = inputs: let
    forAllSystems = function:
      inputs.nixpkgs.lib.genAttrs ["x86_64-linux" "aarch64-linux"] (system:
        function {
          pkgs = import inputs.nixpkgs {
            inherit system;
            overlays = [inputs.frc-nix.overlays.default];
            config.allowUnfree = true;
          };
          inherit system;
        });

    vscodeForSystem = {pkgs, ...}:
      pkgs.vscode-with-extensions.override {
        vscodeExtensions =
          (with pkgs.vscode-extensions; [
            redhat.java
            vscjava.vscode-gradle
            vscjava.vscode-java-debug
            vscjava.vscode-java-dependency
            wpilibsuite.vscode-wpilib

            jgclark.vscode-todo-highlight

            vscjava.vscode-java-pack # doens't work (for some reason), just supresses prompts to install it
          ])
          ++ pkgs.vscode-utils.extensionsFromVscodeMarketplace [
            {
              name = "vsliveshare";
              publisher = "MS-vsliveshare";
              version = "1.0.5959";
              sha256 = "sha256-MibP2zqTwlXXVsXQOSuoi5SO8BskJC/AihrhJFg8tac=";
            }
          ];
      };
  in {
    devShells = forAllSystems ({pkgs, ...} @ args: {
      default = pkgs.mkShell {
        buildInputs = [(vscodeForSystem args) pkgs.jdk21];
      };
    });

    packages = forAllSystems ({pkgs, ...} @ args: {
      code = vscodeForSystem args;
    });
  };
}
