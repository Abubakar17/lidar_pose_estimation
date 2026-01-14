# Model Directory

## Structure

### exports/
Deployment-ready model for inference on Jetson.

**Essential files:**
- `pose_model.onnx` - ONNX model (28KB)
- `scaler_X.joblib` - Input scaler (4.2KB)
- `scaler_Y.joblib` - Output scaler (759B)
- `metadata.json` - Model metadata
- `inference_onnx.py` - Inference script

### outputs/
Training artifacts and visualizations.

**Results:**
- `metrics.json` - Training metrics
- `training_results.png` - Loss curves
- `pose_model.pt` - PyTorch checkpoint

**Visualizations:**
- `pose_comparison.html` - Interactive pose comparison
- `lidar_3d_interactive.html` - 3D point cloud viewer
- `lidar_scan_*.png` - Static visualizations

### training_data/
Original training dataset (23MB total).
- `train_X.csv` - Input features
- `train_Y.csv` - Ground truth poses

## Usage
```python
from exports.inference_onnx import PoseEstimator

estimator = PoseEstimator('exports/', use_gpu=False)
pose = estimator.predict(features)
```
