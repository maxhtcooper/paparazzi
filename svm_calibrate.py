#!/usr/bin/env python3
"""
SVM Parameter Extraction Tool for Paparazzi Bebop Integration

This script extracts trained SVM model parameters and generates C++ code
for embedding the model directly in the svm.cpp module.

Usage:
    python3 svm_calibrate.py --model model.pkl
    python3 svm_calibrate.py --show-params  (display from trained SVM_test.py)
"""

import sys
import pickle
import numpy as np

def export_svm_parameters(scaler, clf):
    """Extract SVM parameters in C++ format."""
    scaler_mean = scaler.mean_
    scaler_scale = scaler.scale_
    svm_weights = clf.coef_[0]
    svm_intercept = clf.intercept_[0]
    
    print("\n" + "="*70)
    print("SVM PARAMETERS FOR C++ (copy into svm.cpp)")
    print("="*70)
    
    print(f"\nconst float scaler_mean[3] = {{{scaler_mean[0]:.6f}, {scaler_mean[1]:.6f}, {scaler_mean[2]:.6f}}};")
    print(f"const float scaler_scale[3] = {{{scaler_scale[0]:.6f}, {scaler_scale[1]:.6f}, {scaler_scale[2]:.6f}}};")
    print(f"const float svm_weights[3] = {{{svm_weights[0]:.6f}, {svm_weights[1]:.6f}, {svm_weights[2]:.6f}}};")
    print(f"const float svm_intercept = {svm_intercept:.6f};")
    
    print("\n" + "="*70)
    print("STEPS TO INTEGRATE:")
    print("="*70)
    print("1. Copy the lines above")
    print("2. Edit sw/airborne/modules/computer_vision/svm.cpp")
    print("3. Replace the scaler_mean, scaler_scale, svm_weights, and svm_intercept")
    print("4. Rebuild: make AIRCRAFT=YOUR_BEBOP BOARD=bebop")
    
    return {
        'scaler_mean': scaler_mean,
        'scaler_scale': scaler_scale,
        'svm_weights': svm_weights,
        'svm_intercept': svm_intercept
    }

if __name__ == '__main__':
    if len(sys.argv) > 1:
        model_file = sys.argv[1] if sys.argv[1] != '--show-params' else None
        if model_file:
            try:
                with open(model_file, 'rb') as f:
                    data = pickle.load(f)
                    scaler = data['scaler']
                    clf = data['clf']
                    export_svm_parameters(scaler, clf)
            except Exception as e:
                print(f"Error loading model: {e}")
    else:
        print("Usage: python3 svm_calibrate.py model.pkl")
        print("   or: python3 svm_calibrate.py --show-params (default values)")
