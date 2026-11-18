#!/usr/bin/env python3
"""Environment and capability verification script for SENTINEL."""

import sys
import os

def check_environment():
    print("=" * 60)
    print("SENTINEL Environment Diagnostics")
    print("=" * 60)

    # 1. Python version
    print(f"Python Executable : {sys.executable}")
    print(f"Python Version    : {sys.version.split()[0]}")
    in_venv = sys.prefix != sys.base_prefix
    print(f"Virtual Env Active: {'YES' if in_venv else 'NO (Recommended: source .venv/bin/activate)'}")
    
    # 2. Check Core Dependencies
    dependencies = [
        ("cv2", "OpenCV"),
        ("numpy", "NumPy"),
        ("fastapi", "FastAPI"),
        ("uvicorn", "Uvicorn"),
        ("websockets", "WebSockets"),
        ("yaml", "PyYAML"),
        ("pydantic", "Pydantic"),
    ]

    all_ok = True
    print("\n--- Package Verification ---")
    for mod_name, label in dependencies:
        try:
            mod = __import__(mod_name)
            ver = getattr(mod, "__version__", "Available")
            print(f"[OK] {label:<15}: {ver}")
        except ImportError as e:
            print(f"[FAIL] {label:<15}: Missing ({e})")
            all_ok = False

    # 3. Check OpenCV ArUco Module
    print("\n--- OpenCV ArUco Check ---")
    try:
        import cv2
        if hasattr(cv2, "aruco") and hasattr(cv2.aruco, "getPredefinedDictionary"):
            print("[OK] cv2.aruco module is available.")
            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            print("[OK] DICT_4X4_50 dictionary successfully loaded.")
        else:
            print("[FAIL] cv2.aruco is not available in this OpenCV build.")
            all_ok = False
    except Exception as e:
        print(f"[FAIL] OpenCV ArUco check error: {e}")
        all_ok = False

    # 4. Probe Camera Indices (Quick non-blocking check)
    print("\n--- Camera Index Probing ---")
    found_cameras = []
    try:
        import cv2
        for idx in range(3):
            cap = cv2.VideoCapture(idx)
            if cap.isOpened():
                ret, _ = cap.read()
                w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                cap.release()
                if ret:
                    found_cameras.append((idx, w, h))
                    print(f"[FOUND] Camera Index {idx}: Active ({w}x{h})")
                else:
                    print(f"[OPEN] Camera Index {idx}: Opened but could not read test frame")
            else:
                print(f"[INFO] Camera Index {idx}: Not detected / busy")
    except Exception as e:
        print(f"[WARN] Camera probe encountered: {e}")

    print("\n" + "=" * 60)
    if all_ok:
        print("RESULT: ALL CORE CAPABILITIES CONFIRMED. READY FOR MILESTONE 2.")
    else:
        print("RESULT: ENVIRONMENT SETUP INCOMPLETE. CHECK ERRORS ABOVE.")
    print("=" * 60)
    return all_ok

if __name__ == "__main__":
    success = check_environment()
    sys.exit(0 if success else 1)
