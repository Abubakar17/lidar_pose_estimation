"""
Pose Estimation Inference Script (ONNX Runtime)
================================================
Standalone inference using ONNX Runtime.
Works on any platform with onnxruntime installed.

Usage:
    from inference_onnx import PoseEstimator
    
    estimator = PoseEstimator("exports")
    pose = estimator.predict(features)  # features: (150,) or (N, 150)
"""

import numpy as np
import json
from pathlib import Path

try:
    import onnxruntime as ort
except ImportError:
    raise ImportError("Please install onnxruntime: pip install onnxruntime")


class PoseEstimator:
    """
    6DoF Pose Estimator using ONNX Runtime.
    
    Attributes:
        session: ONNX Runtime inference session
        X_mean, X_scale: Input normalization parameters
        Y_mean, Y_scale: Output denormalization parameters
    """
    
    def __init__(self, model_dir: str, use_gpu: bool = False):
        """
        Initialize the pose estimator.
        
        Args:
            model_dir: Directory containing model files
            use_gpu: Whether to use GPU (requires onnxruntime-gpu)
        """
        model_dir = Path(model_dir)
        
        # Load ONNX model
        model_path = model_dir / "pose_model.onnx"
        
        if use_gpu:
            providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
        else:
            providers = ['CPUExecutionProvider']
        
        self.session = ort.InferenceSession(str(model_path), providers=providers)
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name
        
        # Load scalers
        scalers = np.load(model_dir / "scalers.npz")
        self.X_mean = scalers['X_mean']
        self.X_scale = scalers['X_scale']
        self.Y_mean = scalers['Y_mean']
        self.Y_scale = scalers['Y_scale']
        
        # Load metadata
        with open(model_dir / "metadata.json") as f:
            self.metadata = json.load(f)
        
        print(f"PoseEstimator initialized")
        print(f"  Model: {model_path}")
        print(f"  Provider: {self.session.get_providers()[0]}")
    
    def preprocess(self, features: np.ndarray) -> np.ndarray:
        """Normalize input features."""
        features = np.asarray(features, dtype=np.float32)
        if features.ndim == 1:
            features = features.reshape(1, -1)
        return (features - self.X_mean) / self.X_scale
    
    def postprocess(self, output: np.ndarray) -> np.ndarray:
        """Denormalize output to original scale."""
        return output * self.Y_scale + self.Y_mean
    
    def predict(self, features: np.ndarray) -> np.ndarray:
        """
        Predict 6DoF pose from LiDAR features.
        
        Args:
            features: Input features, shape (150,) or (N, 150)
        
        Returns:
            pose: Predicted pose [x, y, z, roll, pitch, yaw]
                  Position in meters, angles in radians
                  Shape: (6,) or (N, 6)
        """
        # Preprocess
        features_normalized = self.preprocess(features)
        
        # Run inference
        output = self.session.run(
            [self.output_name],
            {self.input_name: features_normalized}
        )[0]
        
        # Postprocess
        pose = self.postprocess(output)
        
        # Return single pose if single input
        if pose.shape[0] == 1:
            return pose[0]
        return pose
    
    def predict_with_units(self, features: np.ndarray) -> dict:
        """
        Predict pose and return as dictionary with units.
        
        Returns:
            dict with keys: x_m, y_m, z_m, roll_deg, pitch_deg, yaw_deg
        """
        pose = self.predict(features)
        
        if pose.ndim == 1:
            return {
                'x_m': float(pose[0]),
                'y_m': float(pose[1]),
                'z_m': float(pose[2]),
                'roll_deg': float(np.degrees(pose[3])),
                'pitch_deg': float(np.degrees(pose[4])),
                'yaw_deg': float(np.degrees(pose[5])),
            }
        else:
            return {
                'x_m': pose[:, 0],
                'y_m': pose[:, 1],
                'z_m': pose[:, 2],
                'roll_deg': np.degrees(pose[:, 3]),
                'pitch_deg': np.degrees(pose[:, 4]),
                'yaw_deg': np.degrees(pose[:, 5]),
            }


# ============================================================================
# EXAMPLE USAGE
# ============================================================================

if __name__ == "__main__":
    import time
    
    # Initialize estimator
    estimator = PoseEstimator(".", use_gpu=False)
    
    # Create dummy input (replace with real LiDAR features)
    dummy_features = np.random.randn(150).astype(np.float32)
    
    # Single prediction
    print("\nSingle prediction:")
    pose = estimator.predict(dummy_features)
    print(f"  Pose: x={pose[0]*100:.1f}cm, y={pose[1]*100:.1f}cm, z={pose[2]*100:.1f}cm")
    print(f"        roll={np.degrees(pose[3]):.2f}°, pitch={np.degrees(pose[4]):.2f}°, yaw={np.degrees(pose[5]):.2f}°")
    
    # With units
    print("\nWith units:")
    result = estimator.predict_with_units(dummy_features)
    print(f"  {result}")
    
    # Batch prediction
    print("Batch prediction (100 samples):")
    batch_features = np.random.randn(100, 150).astype(np.float32)
    poses = estimator.predict(batch_features)
    print(f"  Output shape: {poses.shape}")
    
    # Benchmark
    print("\nBenchmark (1000 iterations):")
    start = time.time()
    for _ in range(1000):
        _ = estimator.predict(dummy_features)
    elapsed = time.time() - start
    print(f"  Total: {elapsed*1000:.1f}ms")
    print(f"  Per inference: {elapsed:.3f}ms")
    print(f"  Throughput: {1000/elapsed:.0f} predictions/sec")
