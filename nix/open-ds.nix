{
  pkgs,
  jdk ? pkgs.jdk8,
  src,
}:
pkgs.stdenv.mkDerivation {
  pname = "open-ds";
  version = "0.3.1";

  inherit src;

  dontUnpack = true;

  nativeBuildInputs = [pkgs.makeWrapper pkgs.unzip];

  buildInputs = with pkgs; [
    xorg.libX11
    xorg.libXt
    xorg.libXtst
    xorg.libXinerama
    xorg.libXi
    xorg.libXrandr
    xorg.libXrender
    xorg.libXfixes
    libxkbcommon
    gtk3
    gtk2
    glib
  ];

  installPhase = ''
    mkdir -p $out/bin $out/share/java

    cp $src $out/share/java/open-ds.jar

    cat > $out/bin/open-ds <<'EOF'
    #!/bin/sh
    set -e

    # Create a temporary directory for this run
    RUNTIME_DIR="''${XDG_RUNTIME_DIR:-''${TMPDIR:-/tmp}}/opends-$$"
    mkdir -p "$RUNTIME_DIR"

    # Clean up on exit
    cleanup() {
      rm -rf "$RUNTIME_DIR"
    }
    trap cleanup EXIT INT TERM

    # Copy JAR to writable location
    cp "${placeholder "out"}/share/java/open-ds.jar" "$RUNTIME_DIR/open-ds.jar"

    # Set up library path for JNativeHook dependencies
    export LD_LIBRARY_PATH="${pkgs.lib.makeLibraryPath (with pkgs; [xorg.libX11 xorg.libXt xorg.libXtst xorg.libXinerama xorg.libXi xorg.libXrandr xorg.libXrender xorg.libXfixes libxkbcommon gtk3 gtk2 glib])}:''${LD_LIBRARY_PATH:-}"

    # Run from the writable location
    exec "${jdk}/bin/java" \
      -Dswing.defaultlaf=javax.swing.plaf.metal.MetalLookAndFeel \
      -Dswing.metalTheme=steel \
      -Dawt.useSystemAAFontSettings=on \
      -Dswing.aatext=true \
      -jar "$RUNTIME_DIR/open-ds.jar" "$@"
    EOF
    chmod +x $out/bin/open-ds
  '';
}
