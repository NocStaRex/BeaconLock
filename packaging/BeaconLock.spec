# -*- mode: python ; coding: utf-8 -*-
"""
packaging/BeaconLock.spec
=========================
Lean PyInstaller standalone executable specification for BeaconLock.
ISRO SIH-2026 | PS-26169 | Team AlphaTrion (ID: 176697)
"""

import os
from pathlib import Path

# Project root (one directory above packaging/)
PROJECT_ROOT = os.path.abspath(os.path.join(SPECPATH, ".."))

datas = [
    (os.path.join(PROJECT_ROOT, "ui", "assets", "icon.ico"), os.path.join("ui", "assets")),
    (os.path.join(PROJECT_ROOT, "ui", "assets", "icon.png"), os.path.join("ui", "assets")),
]

hiddenimports = [
    "reportlab",
    "reportlab.lib",
    "reportlab.lib.colors",
    "reportlab.lib.pagesizes",
    "reportlab.lib.styles",
    "reportlab.lib.units",
    "reportlab.platypus",
    "reportlab.platypus.paragraph",
    "reportlab.platypus.tables",
    "PySide6",
    "PySide6.QtCore",
    "PySide6.QtGui",
    "PySide6.QtWidgets",
    "pyqtgraph",
    "cv2",
    "numpy",
    "filterpy",
    "filterpy.kalman",
    "pandas",
    "yaml",
    "PIL",
]

# Exclude unnecessary dev/test dependencies for a lean binary
excludes = [
    "tests",
    "pytest",
    "unittest",
    "tkinter",
    "matplotlib",
    "scipy",
    "IPython",
    "jupyter",
    "sqlite3",
]

a = Analysis(
    [os.path.join(PROJECT_ROOT, "main.py")],
    pathex=[PROJECT_ROOT],
    binaries=[],
    datas=datas,
    hiddenimports=hiddenimports,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=excludes,
    noarchive=False,
    optimize=1,
)

pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.datas,
    [],
    name="BeaconLock",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    icon=os.path.join(PROJECT_ROOT, "ui", "assets", "icon.ico"),
)
