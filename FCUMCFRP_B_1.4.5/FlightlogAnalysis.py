"""
Log Analysis Tool for FCU IMU Telemetry Data
Analyzes saved CSV log files and generates comprehensive plots:

Enhanced with Event Detection and Disturbance Analysis:
1. Mean Absolute Error (MAE)
2. Root Mean Square Error (RMSE)
3. Standard Deviation of Error (σ)
4. Settling Time (2% and 5% tolerance)
5. Signal-to-Noise Ratio (SNR)
6. Peak-to-Peak Error
7. Error Distribution Analysis (Skewness, Kurtosis)
8. 95th Percentile Error
9. Stability Index
10. Convergence Time Analysis
11. Drift Rate Detection
12. Frequency Domain Analysis
13. Cross-Correlation Metrics
14. Event Pattern Analysis
15. Disturbance Detection Statistics

Author: Patrick Andrasena T.
Version: 2.0 (Enhanced with Event Detection Analysis)

============================================================
ENHANCED FEATURES - EVENT DETECTION ANALYSIS
============================================================

This enhanced version includes comprehensive event detection and disturbance analysis:

1. EVENT ANALYSIS:
   - Command input pattern analysis
   - Disturbance event detection and statistics
   - Event frequency and timing analysis
   - Command vs. disturbance correlation studies

2. DISTURBANCE METRICS:
   - Disturbance count and frequency
   - Disturbance magnitude distribution
   - Time between disturbances analysis
   - Disturbance response time metrics

3. ADVANCED STABILITY METRICS:
   - Signal-to-Noise Ratio (SNR) in dB
   - Peak-to-Peak error analysis
   - Error distribution statistics (skewness, kurtosis)
   - 95th percentile error for robust assessment
   - Stability index (std/MAE ratio)
   - Convergence rate analysis
   - Drift rate detection (linear trends)
   - Frequency domain analysis (dominant frequencies)

4. DATA INTEGRITY VALIDATION:
   - UDP packet structure validation
   - Missing value detection
   - Out-of-range value checking
   - Timestamp gap analysis (packet loss)
   - Sensor anomaly detection
   - Overall data quality scoring (0-100)

5. CROSS-CORRELATION ANALYSIS:
   - Control vs. Monitor IMU correlation
   - Raw sensor correlation analysis
   - Attitude estimation consistency
   - Sensor calibration validation

6. COMPREHENSIVE REPORTING:
   - Timestamped report directories
   - Multiple plot types (5 different visualizations)
   - Detailed text reports with metrics
   - Event timeline and pattern analysis
   - Quality and stability level classification

7. RESEARCH APPLICATIONS:
   - Flight control system validation
   - Disturbance rejection algorithm evaluation
   - Wind gust response characterization
   - Control system robustness assessment
   - Sensor fusion performance analysis

8. OUTPUT STRUCTURE:
   - report_YYYYMMDD_HHMMSS/
     ├── comprehensive_stability_report.txt
     ├── plots/
     │   ├── main_analysis.png
     │   ├── error_analysis.png
     │   ├── data_integrity.png
     │   ├── correlation_heatmap.png
     │   └── frequency_analysis.png
     └── data/
         └── original_data.csv

============================================================
USAGE
============================================================

1. Run the analysis tool:
   python FlightlogAnalysis.py

2. Select from available log files (timestamped directories)

3. The tool will:
   - Validate data integrity and quality
   - Calculate comprehensive stability metrics
   - Perform event pattern analysis
   - Generate multiple visualization plots
   - Create detailed analysis reports
   - Provide quality and stability assessments

4. Use the generated reports for:
   - Flight control system research
   - Algorithm validation and tuning
   - Performance benchmarking
   - Disturbance response studies

"""

import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime
from scipy import stats
import warnings
warnings.filterwarnings('ignore')

# Configuration
DATA_LOGS_DIR = "data_logs"
CSV_FILENAME = "imu_data.csv"


def scan_log_directories():
    """Scan data_logs directory for timestamped folders"""
    if not os.path.exists(DATA_LOGS_DIR):
        print(f"Error: Directory '{DATA_LOGS_DIR}' not found.")
        return []
    
    # Find all timestamped directories (format: YYYYMMDD_HHMMSS)
    log_dirs = []
    for item in os.listdir(DATA_LOGS_DIR):
        item_path = os.path.join(DATA_LOGS_DIR, item)
        if os.path.isdir(item_path):
            csv_path = os.path.join(item_path, CSV_FILENAME)
            if os.path.exists(csv_path):
                log_dirs.append(item)
    
    # Sort by timestamp (newest first)
    log_dirs.sort(reverse=True)
    return log_dirs


def list_log_files():
    """List available log files with details"""
    log_dirs = scan_log_directories()
    
    if not log_dirs:
        print("No log files found in data_logs directory.")
        return None
    
    print("\n" + "=" * 70)
    print("Available Log Files:")
    print("=" * 70)
    
    log_info = []
    for idx, log_dir in enumerate(log_dirs, 1):
        csv_path = os.path.join(DATA_LOGS_DIR, log_dir, CSV_FILENAME)
        
        # Get file stats
        try:
            file_size = os.path.getsize(csv_path)
            file_size_mb = file_size / (1024 * 1024)
            
            # Count lines in CSV
            with open(csv_path, 'r') as f:
                line_count = sum(1 for line in f) - 1  # Subtract header
            
            # Parse timestamp
            try:
                dt = datetime.strptime(log_dir, "%Y%m%d_%H%M%S")
                timestamp_str = dt.strftime("%Y-%m-%d %H:%M:%S")
            except:
                timestamp_str = log_dir
            
            log_info.append({
                'index': idx,
                'directory': log_dir,
                'path': csv_path,
                'timestamp': timestamp_str,
                'size_mb': file_size_mb,
                'rows': line_count
            })
            
            print(f"  [{idx:2d}] {timestamp_str} | {line_count:6d} rows | {file_size_mb:.2f} MB")
            
        except Exception as e:
            print(f"  [{idx:2d}] {log_dir} | Error reading file: {e}")
    
    print("=" * 70)
    return log_info


def select_log_file():
    """Interactive log file selection"""
    log_info = list_log_files()
    
    if not log_info:
        return None
    
    while True:
        try:
            choice = input(f"\nSelect log file [1-{len(log_info)}] (or 'q' to quit): ").strip()
            
            if choice.lower() == 'q':
                return None
            
            choice_idx = int(choice)
            if 1 <= choice_idx <= len(log_info):
                selected = log_info[choice_idx - 1]
                print(f"\nSelected: {selected['timestamp']} ({selected['rows']} rows)")
                return selected['path']
            else:
                print(f"Invalid choice. Please enter a number between 1 and {len(log_info)}.")
                
        except ValueError:
            print("Invalid input. Please enter a number or 'q' to quit.")
        except KeyboardInterrupt:
            print("\nCancelled.")
            return None


def load_csv_data(csv_path):
    """Load CSV data into pandas DataFrame"""
    try:
        print(f"\nLoading data from: {csv_path}")
        df = pd.read_csv(csv_path)
        
        # Convert timestamp_us to relative time in seconds
        if 'timestamp_us' in df.columns and len(df) > 0:
            first_timestamp = df['timestamp_us'].iloc[0]
            df['time_s'] = (df['timestamp_us'] - first_timestamp) / 1e6
        
        print(f"Loaded {len(df)} data points")
        print(f"Time span: {df['time_s'].iloc[-1]:.2f} seconds")
        
        return df
        
    except Exception as e:
        print(f"Error loading CSV file: {e}")
        return None


def create_analysis_plots(df):
    """Create comprehensive analysis plots"""
    if df is None or len(df) == 0:
        print("No data to plot.")
        return
    
    # Create figure with 2 rows, 2 columns (same layout as UDPDataClient)
    fig, axes = plt.subplots(2, 2, figsize=(16, 10))
    fig.suptitle('FCU IMU Telemetry - Log Analysis', fontsize=16, fontweight='bold')
    
    time_data = df['time_s'].values
    
    # Plot 1: Attitude Comparison (top-left)
    ax1 = axes[0, 0]
    ax1.set_title('Attitude Comparison (Roll, Pitch, Yaw)', fontsize=12, fontweight='bold')
    ax1.set_xlabel('Time (s)', fontsize=10)
    ax1.set_ylabel('Angle (deg)', fontsize=10)
    ax1.grid(True, alpha=0.3)
    
    ax1.plot(time_data, df['ctrl_roll'], 'r-', label='Control Roll', linewidth=1.5)
    ax1.plot(time_data, df['ctrl_pitch'], 'g-', label='Control Pitch', linewidth=1.5)
    ax1.plot(time_data, df['ctrl_yaw'], 'b-', label='Control Yaw', linewidth=1.5)
    ax1.plot(time_data, df['mon_roll'], 'r--', label='Monitor Roll', linewidth=1.5, alpha=0.7)
    ax1.plot(time_data, df['mon_pitch'], 'g--', label='Monitor Pitch', linewidth=1.5, alpha=0.7)
    ax1.plot(time_data, df['mon_yaw'], 'b--', label='Monitor Yaw', linewidth=1.5, alpha=0.7)
    ax1.legend(loc='upper right', fontsize=8)
    
    # Plot 2: Attitude Error (top-right)
    ax2 = axes[0, 1]
    ax2.set_title('Attitude Error (Control - Monitor)', fontsize=12, fontweight='bold')
    ax2.set_xlabel('Time (s)', fontsize=10)
    ax2.set_ylabel('Error (deg)', fontsize=10)
    ax2.grid(True, alpha=0.3)
    ax2.axhline(y=0, color='k', linestyle='--', alpha=0.5)
    
    ax2.plot(time_data, df['err_roll'], 'r-', label='Roll Error', linewidth=1.5)
    ax2.plot(time_data, df['err_pitch'], 'g-', label='Pitch Error', linewidth=1.5)
    ax2.plot(time_data, df['err_yaw'], 'b-', label='Yaw Error', linewidth=1.5)
    ax2.legend(loc='upper right', fontsize=8)
    
    # Calculate and display error statistics (old code)
    # err_roll_std = df['err_roll'].std()
    # err_pitch_std = df['err_pitch'].std()
    # err_yaw_std = df['err_yaw'].std()
    # err_roll_max = df['err_roll'].abs().max()
    # err_pitch_max = df['err_pitch'].abs().max()
    # err_yaw_max = df['err_yaw'].abs().max()
    
    # Calculate stability metrics
    metrics = calculate_stability_metrics(df)
    
    # Replace with:
    err_roll_max = df['err_roll'].abs().max()
    err_pitch_max = df['err_pitch'].abs().max()
    err_yaw_max = df['err_yaw'].abs().max()
    
    # stats_text = f"Roll:  σ={err_roll_std:.3f}°, max={err_roll_max:.3f}°\n"
    # stats_text += f"       MAE={metrics['roll']['mae']:.3f}°, RMSE={metrics['roll']['rmse']:.3f}°\n"
    # stats_text += f"Pitch: σ={err_pitch_std:.3f}°, max={err_pitch_max:.3f}°\n"
    # stats_text += f"       MAE={metrics['pitch']['mae']:.3f}°, RMSE={metrics['pitch']['rmse']:.3f}°\n"
    # stats_text += f"Yaw:   σ={err_yaw_std:.3f}°, max={err_yaw_max:.3f}°\n"
    # stats_text += f"       MAE={metrics['yaw']['mae']:.3f}°, RMSE={metrics['yaw']['rmse']:.3f}°"
    
    stats_text = f"Roll:  σ={metrics['roll']['std']:.3f}°, max={err_roll_max:.3f}°\n"
    stats_text += f"       MAE={metrics['roll']['mae']:.3f}°, RMSE={metrics['roll']['rmse']:.3f}°\n"
    stats_text += f"Pitch: σ={metrics['pitch']['std']:.3f}°, max={err_pitch_max:.3f}°\n"
    stats_text += f"       MAE={metrics['pitch']['mae']:.3f}°, RMSE={metrics['pitch']['rmse']:.3f}°\n"
    stats_text += f"Yaw:   σ={metrics['yaw']['std']:.3f}°, max={err_yaw_max:.3f}°\n"
    stats_text += f"       MAE={metrics['yaw']['mae']:.3f}°, RMSE={metrics['yaw']['rmse']:.3f}°"
    
    # Add settling time if available
    settling_times = []
    for axis in ['roll', 'pitch', 'yaw']:
        st = metrics[axis]['settling_time_2pct']
        if st is not None:
            settling_times.append(f"{axis[0].upper()}:{st:.2f}s")
    
    if settling_times:
        stats_text += f"\n\nSettling (2%): {', '.join(settling_times)}"
    
    ax2.text(0.02, 0.98, stats_text, transform=ax2.transAxes,
             fontsize=7, verticalalignment='top', bbox=dict(boxstyle='round', 
             facecolor='wheat', alpha=0.5))
    
    # Draw settling time markers on error plot
    tolerance_2pct = 0.02
    tolerance_5pct = 0.05
    for axis, color in [('roll', 'r'), ('pitch', 'g'), ('yaw', 'b')]:
        error_data = df[f'err_{axis}'].values
        max_abs_error = np.abs(error_data).max()
        tol_2pct = tolerance_2pct * max_abs_error
        tol_5pct = tolerance_5pct * max_abs_error
        
        # Draw tolerance bands
        ax2.axhline(y=tol_2pct, color=color, linestyle=':', alpha=0.3, linewidth=1)
        ax2.axhline(y=-tol_2pct, color=color, linestyle=':', alpha=0.3, linewidth=1)
        
        # Mark settling time
        st_2pct = metrics[axis]['settling_time_2pct']
        if st_2pct is not None:
            ax2.axvline(x=st_2pct, color=color, linestyle='--', alpha=0.5, linewidth=1.5)
    
    # Plot 3: Accelerometer Data (bottom-left)
    ax3 = axes[1, 0]
    ax3.set_title('Accelerometer Data (g)', fontsize=12, fontweight='bold')
    ax3.set_xlabel('Time (s)', fontsize=10)
    ax3.set_ylabel('Acceleration (g)', fontsize=10)
    ax3.grid(True, alpha=0.3)
    
    ax3.plot(time_data, df['ctrl_acc_x'], 'r-', label='Control X', linewidth=1.5)
    ax3.plot(time_data, df['ctrl_acc_y'], 'g-', label='Control Y', linewidth=1.5)
    ax3.plot(time_data, df['ctrl_acc_z'], 'b-', label='Control Z', linewidth=1.5)
    ax3.plot(time_data, df['mon_acc_x'], 'r--', label='Monitor X', linewidth=1.5, alpha=0.7)
    ax3.plot(time_data, df['mon_acc_y'], 'g--', label='Monitor Y', linewidth=1.5, alpha=0.7)
    ax3.plot(time_data, df['mon_acc_z'], 'b--', label='Monitor Z', linewidth=1.5, alpha=0.7)
    ax3.legend(loc='upper right', fontsize=8)
    
    # Plot 4: Gyroscope Data (bottom-right)
    ax4 = axes[1, 1]
    ax4.set_title('Gyroscope Data (deg/s)', fontsize=12, fontweight='bold')
    ax4.set_xlabel('Time (s)', fontsize=10)
    ax4.set_ylabel('Angular Rate (deg/s)', fontsize=10)
    ax4.grid(True, alpha=0.3)
    
    ax4.plot(time_data, df['ctrl_gyro_x'], 'r-', label='Control X', linewidth=1.5)
    ax4.plot(time_data, df['ctrl_gyro_y'], 'g-', label='Control Y', linewidth=1.5)
    ax4.plot(time_data, df['ctrl_gyro_z'], 'b-', label='Control Z', linewidth=1.5)
    ax4.plot(time_data, df['mon_gyro_x'], 'r--', label='Monitor X', linewidth=1.5, alpha=0.7)
    ax4.plot(time_data, df['mon_gyro_y'], 'g--', label='Monitor Y', linewidth=1.5, alpha=0.7)
    ax4.plot(time_data, df['mon_gyro_z'], 'b--', label='Monitor Z', linewidth=1.5, alpha=0.7)
    ax4.legend(loc='upper right', fontsize=8)
    
    plt.tight_layout()
    
    return fig


def calculate_mae(series):
    """Calculate Mean Absolute Error"""
    return series.abs().mean()

def calculate_std(series):
    """Calculate Standard Deviation"""
    return series.std()

def calculate_rmse(series):
    """Calculate Root Mean Square Error"""
    return np.sqrt((series ** 2).mean())


def calculate_settling_time(time_data, error_data, tolerance_percent=2.0, window_size=50):
    """
    Calculate settling time - time for error to reach and stay within tolerance
    
    Args:
        time_data: Time array
        error_data: Error signal array
        tolerance_percent: Percentage tolerance (default 2%)
        window_size: Number of samples to check for stability
    
    Returns:
        Settling time in seconds, or None if not settled
    """
    if len(error_data) < window_size:
        return None
    
    # Calculate tolerance based on maximum absolute error
    max_abs_error = np.abs(error_data).max()
    tolerance = tolerance_percent / 100.0 * max_abs_error
    
    # Minimum tolerance to avoid false positives with very small errors
    min_tolerance = 0.01  # degrees
    tolerance = max(tolerance, min_tolerance)
    
    # Find when error enters and stays within tolerance
    # Check if error stays within tolerance for the entire window
    for i in range(len(error_data) - window_size):
        window = error_data[i:i + window_size]
        if np.all(np.abs(window) <= tolerance):
            return time_data[i]
    
    return None


def validate_data_integrity(df):
    """Validate data integrity based on UDP packet structure and publisher checks"""
    integrity_report = {
        'total_samples': len(df),
        'missing_values': {},
        'out_of_range_values': {},
        'timestamp_gaps': [],
        'sensor_anomalies': {},
        'data_quality_score': 0.0
    }
    
    # Check for missing values
    for col in df.columns:
        missing_count = df[col].isna().sum()
        if missing_count > 0:
            integrity_report['missing_values'][col] = missing_count
    
    # Check for out-of-range values based on typical IMU specifications
    range_checks = {
        'ctrl_acc_x': (-16, 16), 'ctrl_acc_y': (-16, 16), 'ctrl_acc_z': (-16, 16),
        'mon_acc_x': (-16, 16), 'mon_acc_y': (-16, 16), 'mon_acc_z': (-16, 16),
        'ctrl_gyro_x': (-2000, 2000), 'ctrl_gyro_y': (-2000, 2000), 'ctrl_gyro_z': (-2000, 2000),
        'mon_gyro_x': (-2000, 2000), 'mon_gyro_y': (-2000, 2000), 'mon_gyro_z': (-2000, 2000),
        'ctrl_roll': (-180, 180), 'ctrl_pitch': (-90, 90), 'ctrl_yaw': (-180, 180),
        'mon_roll': (-180, 180), 'mon_pitch': (-90, 90), 'mon_yaw': (-180, 180),
        'err_roll': (-10, 10), 'err_pitch': (-10, 10), 'err_yaw': (-10, 10)
    }
    
    for col, (min_val, max_val) in range_checks.items():
        if col in df.columns:
            out_of_range = df[(df[col] < min_val) | (df[col] > max_val)]
            if len(out_of_range) > 0:
                integrity_report['out_of_range_values'][col] = len(out_of_range)
    
    # Check timestamp gaps (indicative of packet loss)
    if 'timestamp_us' in df.columns:
        time_diffs = df['timestamp_us'].diff()
        expected_interval = time_diffs.median()
        large_gaps = time_diffs[time_diffs > 3 * expected_interval]
        if len(large_gaps) > 0:
            integrity_report['timestamp_gaps'] = large_gaps.tolist()
    
    # Check for sensor anomalies (stuck values, excessive noise)
    sensor_cols = ['ctrl_acc_x', 'ctrl_acc_y', 'ctrl_acc_z', 'ctrl_gyro_x', 'ctrl_gyro_y', 'ctrl_gyro_z']
    for col in sensor_cols:
        if col in df.columns:
            # Check for stuck values (low variance)
            if df[col].var() < 0.001:
                integrity_report['sensor_anomalies'][f'{col}_stuck'] = True
            
            # Check for excessive noise (high variance)
            elif df[col].var() > df[col].mean()**2 * 0.5:
                integrity_report['sensor_anomalies'][f'{col}_noisy'] = True
    
    # Calculate overall data quality score (0-100)
    total_checks = 0
    passed_checks = 0
    
    # Missing values check
    total_checks += 1
    if sum(integrity_report['missing_values'].values()) == 0:
        passed_checks += 1
    
    # Out of range check
    total_checks += 1
    if sum(integrity_report['out_of_range_values'].values()) == 0:
        passed_checks += 1
    
    # Timestamp gap check
    total_checks += 1
    if len(integrity_report['timestamp_gaps']) == 0:
        passed_checks += 1
    
    # Sensor anomaly check
    total_checks += 1
    if len(integrity_report['sensor_anomalies']) == 0:
        passed_checks += 1
    
    integrity_report['data_quality_score'] = (passed_checks / total_checks) * 100
    
    return integrity_report


def calculate_advanced_stability_metrics(df):
    """Calculate advanced stabilization metrics beyond basic error statistics"""
    time_data = df['time_s'].values
    
    metrics = {}
    
    for axis in ['roll', 'pitch', 'yaw']:
        error_col = f'err_{axis}'
        error_data = df[error_col].values
        
        # Basic metrics (existing)
        mae = calculate_mae(df[error_col])
        rmse = calculate_rmse(df[error_col])
        std = calculate_std(df[error_col])
        
        # Advanced metrics
        # 1. Signal-to-Noise Ratio (SNR)
        signal_power = np.mean(error_data**2)
        noise_power = np.var(error_data)
        snr = 10 * np.log10(signal_power / noise_power) if noise_power > 0 else np.inf
        
        # 2. Peak-to-Peak Error
        p2p_error = np.ptp(error_data)
        
        # 3. Error Distribution Skewness and Kurtosis
        skewness = stats.skew(error_data)
        kurtosis = stats.kurtosis(error_data)
        
        # 4. 95th Percentile Error
        percentile_95 = np.percentile(np.abs(error_data), 95)
        
        # 5. Stability Index (ratio of std to mean absolute error)
        stability_index = std / mae if mae > 0 else np.inf
        
        # 6. Convergence Rate (how fast error settles to within 10% of final value)
        final_error_std = np.std(error_data[-100:]) if len(error_data) >= 100 else np.std(error_data)
        convergence_threshold = 0.1 * final_error_std
        convergence_time = None
        for i in range(len(error_data)):
            if np.abs(error_data[i]) <= convergence_threshold:
                convergence_time = time_data[i]
                break
        
        # 7. Drift Rate (linear trend in error)
        if len(time_data) > 1:
            drift_slope, _, _, _, _ = stats.linregress(time_data, error_data)
            drift_rate = drift_slope  # degrees per second
        else:
            drift_rate = 0.0
        
        # 8. Frequency Domain Analysis (dominant frequency in error signal)
        if len(error_data) > 10:
            fft = np.fft.fft(error_data)
            freqs = np.fft.fftfreq(len(error_data), d=(time_data[1] - time_data[0]))
            dominant_freq_idx = np.argmax(np.abs(fft[1:len(fft)//2])) + 1
            dominant_frequency = np.abs(freqs[dominant_freq_idx])
        else:
            dominant_frequency = 0.0
        
        metrics[axis] = {
            'mae': mae,
            'rmse': rmse,
            'std': std,
            'settling_time_2pct': calculate_settling_time(time_data, error_data, tolerance_percent=2.0),
            'settling_time_5pct': calculate_settling_time(time_data, error_data, tolerance_percent=5.0),
            'snr_db': snr,
            'peak_to_peak': p2p_error,
            'skewness': skewness,
            'kurtosis': kurtosis,
            'percentile_95': percentile_95,
            'stability_index': stability_index,
            'convergence_time': convergence_time,
            'drift_rate': drift_rate,
            'dominant_frequency': dominant_frequency
        }
    
    return metrics


def calculate_cross_correlation_metrics(df):
    """Calculate cross-correlation metrics between control and monitor IMUs"""
    correlation_metrics = {}
    
    # Attitude cross-correlation
    for axis in ['roll', 'pitch', 'yaw']:
        ctrl_col = f'ctrl_{axis}'
        mon_col = f'mon_{axis}'
        
        if ctrl_col in df.columns and mon_col in df.columns:
            correlation = np.corrcoef(df[ctrl_col], df[mon_col])[0, 1]
            correlation_metrics[f'attitude_{axis}_correlation'] = correlation
    
    # Raw sensor cross-correlation
    sensor_types = ['acc', 'gyro']
    axes = ['x', 'y', 'z']
    
    for sensor in sensor_types:
        for axis in axes:
            ctrl_col = f'ctrl_{sensor}_{axis}'
            mon_col = f'mon_{sensor}_{axis}'
            
            if ctrl_col in df.columns and mon_col in df.columns:
                correlation = np.corrcoef(df[ctrl_col], df[mon_col])[0, 1]
                correlation_metrics[f'{sensor}_{axis}_correlation'] = correlation
    
    return correlation_metrics


def calculate_stability_metrics(df):
    """Calculate comprehensive stability validation metrics for attitude errors"""
    # Use advanced metrics function
    return calculate_advanced_stability_metrics(df)


def print_data_statistics(df):
    """Print statistical summary of the data"""
    print("\n" + "=" * 70)
    print("Data Statistics:")
    print("=" * 70)
    
    # Calculate stability metrics
    metrics = calculate_stability_metrics(df)
    
    # Attitude statistics
    print("\nAttitude (Control IMU):")
    print(f"  Roll:  mean={df['ctrl_roll'].mean():7.3f}°, std={df['ctrl_roll'].std():7.3f}°, "
          f"min={df['ctrl_roll'].min():7.3f}°, max={df['ctrl_roll'].max():7.3f}°")
    print(f"  Pitch: mean={df['ctrl_pitch'].mean():7.3f}°, std={df['ctrl_pitch'].std():7.3f}°, "
          f"min={df['ctrl_pitch'].min():7.3f}°, max={df['ctrl_pitch'].max():7.3f}°")
    print(f"  Yaw:   mean={df['ctrl_yaw'].mean():7.3f}°, std={df['ctrl_yaw'].std():7.3f}°, "
          f"min={df['ctrl_yaw'].min():7.3f}°, max={df['ctrl_yaw'].max():7.3f}°")
    
    print("\nAttitude (Monitor IMU):")
    print(f"  Roll:  mean={df['mon_roll'].mean():7.3f}°, std={df['mon_roll'].std():7.3f}°, "
          f"min={df['mon_roll'].min():7.3f}°, max={df['mon_roll'].max():7.3f}°")
    print(f"  Pitch: mean={df['mon_pitch'].mean():7.3f}°, std={df['mon_pitch'].std():7.3f}°, "
          f"min={df['mon_pitch'].min():7.3f}°, max={df['mon_pitch'].max():7.3f}°")
    print(f"  Yaw:   mean={df['mon_yaw'].mean():7.3f}°, std={df['mon_yaw'].std():7.3f}°, "
          f"min={df['mon_yaw'].min():7.3f}°, max={df['mon_yaw'].max():7.3f}°")
    
    print("\nAttitude Errors (Control - Monitor):")
    print(f"  Roll:  mean={df['err_roll'].mean():7.3f}°, std={metrics['roll']['std']:7.3f}°, "
          f"max_abs={df['err_roll'].abs().max():7.3f}°")
    print(f"  Pitch: mean={df['err_pitch'].mean():7.3f}°, std={metrics['pitch']['std']:7.3f}°, "
          f"max_abs={df['err_pitch'].abs().max():7.3f}°")
    print(f"  Yaw:   mean={df['err_yaw'].mean():7.3f}°, std={metrics['yaw']['std']:7.3f}°, "
          f"max_abs={df['err_yaw'].abs().max():7.3f}°")
    
    print("\nAccelerometer (Control IMU):")
    print(f"  X: mean={df['ctrl_acc_x'].mean():7.3f}g, std={df['ctrl_acc_x'].std():7.3f}g")
    print(f"  Y: mean={df['ctrl_acc_y'].mean():7.3f}g, std={df['ctrl_acc_y'].std():7.3f}g")
    print(f"  Z: mean={df['ctrl_acc_z'].mean():7.3f}g, std={df['ctrl_acc_z'].std():7.3f}g")
    
    print("\nGyroscope (Control IMU):")
    print(f"  X: mean={df['ctrl_gyro_x'].mean():7.3f}°/s, std={df['ctrl_gyro_x'].std():7.3f}°/s")
    print(f"  Y: mean={df['ctrl_gyro_y'].mean():7.3f}°/s, std={df['ctrl_gyro_y'].std():7.3f}°/s")
    print(f"  Z: mean={df['ctrl_gyro_z'].mean():7.3f}°/s, std={df['ctrl_gyro_z'].std():7.3f}°/s")
    
    print("=" * 70)


def print_data_integrity_report(integrity_report):
    """Print comprehensive data integrity report"""
    print("\n" + "=" * 70)
    print("Data Integrity Validation Report:")
    print("=" * 70)
    
    print(f"\nData Quality Score: {integrity_report['data_quality_score']:.1f}/100")
    print(f"Total Samples: {integrity_report['total_samples']}")
    
    # Missing values
    if integrity_report['missing_values']:
        print("\nMissing Values:")
        for col, count in integrity_report['missing_values'].items():
            print(f"  {col}: {count}")
    else:
        print("\n✓ No missing values detected")
    
    # Out of range values
    if integrity_report['out_of_range_values']:
        print("\nOut-of-Range Values:")
        for col, count in integrity_report['out_of_range_values'].items():
            print(f"  {col}: {count}")
    else:
        print("\n✓ All values within expected ranges")
    
    # Timestamp gaps
    if integrity_report['timestamp_gaps']:
        print(f"\nTimestamp Gaps (packet loss): {len(integrity_report['timestamp_gaps'])}")
        print(f"  Average gap size: {np.mean(integrity_report['timestamp_gaps']):.0f} μs")
    else:
        print("\n✓ No significant timestamp gaps detected")
    
    # Sensor anomalies
    if integrity_report['sensor_anomalies']:
        print("\nSensor Anomalies:")
        for anomaly, flag in integrity_report['sensor_anomalies'].items():
            print(f"  {anomaly}: {flag}")
    else:
        print("\n✓ No sensor anomalies detected")
    
    print("=" * 70)


def print_advanced_stability_metrics(metrics, correlation_metrics=None):
    """Print comprehensive advanced stability metrics"""
    print("\n" + "=" * 70)
    print("Advanced Stability Validation Metrics:")
    print("=" * 70)
    
    for axis in ['roll', 'pitch', 'yaw']:
        print(f"\n{axis.upper()} Axis:")
        print("  " + "-" * 50)
        
        m = metrics[axis]
        
        # Basic metrics
        print(f"  Basic Metrics:")
        print(f"    MAE: {m['mae']:.4f}°")
        print(f"    RMSE: {m['rmse']:.4f}°")
        print(f"    Std Dev: {m['std']:.4f}°")
        print(f"    Peak-to-Peak: {m['peak_to_peak']:.4f}°")
        print(f"    95th Percentile: {m['percentile_95']:.4f}°")
        
        # Advanced metrics
        print(f"  Advanced Metrics:")
        print(f"    Signal-to-Noise Ratio: {m['snr_db']:.2f} dB")
        print(f"    Stability Index: {m['stability_index']:.3f}")
        print(f"    Skewness: {m['skewness']:.3f}")
        print(f"    Kurtosis: {m['kurtosis']:.3f}")
        print(f"    Drift Rate: {m['drift_rate']:.6f}°/s")
        print(f"    Dominant Frequency: {m['dominant_frequency']:.3f} Hz")
        
        # Time-based metrics
        print(f"  Time-Based Metrics:")
        if m['convergence_time'] is not None:
            print(f"    Convergence Time: {m['convergence_time']:.3f} s")
        else:
            print(f"    Convergence Time: Not converged")
        
        if m['settling_time_2pct'] is not None:
            print(f"    Settling Time (2%): {m['settling_time_2pct']:.3f} s")
        else:
            print(f"    Settling Time (2%): Not settled")
        
        if m['settling_time_5pct'] is not None:
            print(f"    Settling Time (5%): {m['settling_time_5pct']:.3f} s")
        else:
            print(f"    Settling Time (5%): Not settled")
    
    # Cross-correlation metrics
    if correlation_metrics:
        print("\n" + "=" * 70)
        print("Cross-Correlation Metrics (Control vs Monitor):")
        print("=" * 70)
        
        print("\nAttitude Correlation:")
        for axis in ['roll', 'pitch', 'yaw']:
            key = f'attitude_{axis}_correlation'
            if key in correlation_metrics:
                print(f"  {axis.capitalize()}: {correlation_metrics[key]:.4f}")
        
        print("\nSensor Correlation:")
        for sensor in ['acc', 'gyro']:
            for axis in ['x', 'y', 'z']:
                key = f'{sensor}_{axis}_correlation'
                if key in correlation_metrics:
                    print(f"  {sensor.upper()} {axis.upper()}: {correlation_metrics[key]:.4f}")
    
    print("=" * 70)
    
    return metrics


def create_report_directory(csv_path):
    """Create timestamped report directory with subdirectories"""
    csv_dir = os.path.dirname(csv_path)
    
    # Create main report directory with timestamp
    report_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    report_dir = os.path.join(csv_dir, f"report_{report_timestamp}")
    os.makedirs(report_dir, exist_ok=True)
    
    # Create subdirectories
    plots_dir = os.path.join(report_dir, "plots")
    os.makedirs(plots_dir, exist_ok=True)
    
    data_dir = os.path.join(report_dir, "data")
    os.makedirs(data_dir, exist_ok=True)
    
    return report_dir, plots_dir, data_dir


def save_enhanced_plots(fig, df, plots_dir, metrics, integrity_report):
    """Save multiple enhanced plot variations"""
    if fig is None:
        return None
    
    saved_plots = {}
    
    # 1. Main analysis plot
    main_plot_path = os.path.join(plots_dir, "main_analysis.png")
    fig.savefig(main_plot_path, dpi=300, bbox_inches='tight', facecolor='white')
    saved_plots['main'] = main_plot_path
    
    # 2. Error analysis plot (enhanced)
    error_fig = create_error_analysis_plot(df, metrics)
    if error_fig:
        error_plot_path = os.path.join(plots_dir, "error_analysis.png")
        error_fig.savefig(error_plot_path, dpi=300, bbox_inches='tight', facecolor='white')
        saved_plots['error_analysis'] = error_plot_path
        plt.close(error_fig)
    
    # 3. Data integrity plot
    integrity_fig = create_integrity_plot(df, integrity_report)
    if integrity_fig:
        integrity_plot_path = os.path.join(plots_dir, "data_integrity.png")
        integrity_fig.savefig(integrity_plot_path, dpi=300, bbox_inches='tight', facecolor='white')
        saved_plots['integrity'] = integrity_plot_path
        plt.close(integrity_fig)
    
    # 4. Correlation heatmap
    correlation_fig = create_correlation_heatmap(df)
    if correlation_fig:
        corr_plot_path = os.path.join(plots_dir, "correlation_heatmap.png")
        correlation_fig.savefig(corr_plot_path, dpi=300, bbox_inches='tight', facecolor='white')
        saved_plots['correlation'] = corr_plot_path
        plt.close(correlation_fig)
    
    # 5. Frequency analysis plot
    freq_fig = create_frequency_analysis_plot(df, metrics)
    if freq_fig:
        freq_plot_path = os.path.join(plots_dir, "frequency_analysis.png")
        freq_fig.savefig(freq_plot_path, dpi=300, bbox_inches='tight', facecolor='white')
        saved_plots['frequency'] = freq_plot_path
        plt.close(freq_fig)
    
    return saved_plots


def create_error_analysis_plot(df, metrics):
    """Create detailed error analysis plot"""
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    fig.suptitle('Detailed Error Analysis', fontsize=16, fontweight='bold')
    
    time_data = df['time_s'].values
    
    # Plot 1: Error distributions
    ax1 = axes[0, 0]
    ax1.set_title('Error Distribution Histograms', fontsize=12, fontweight='bold')
    
    for axis, color in [('roll', 'r'), ('pitch', 'g'), ('yaw', 'b')]:
        error_data = df[f'err_{axis}'].values
        ax1.hist(error_data, bins=50, alpha=0.6, color=color, label=f'{axis.capitalize()} Error', density=True)
    
    ax1.set_xlabel('Error (deg)')
    ax1.set_ylabel('Probability Density')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # Plot 2: Cumulative error
    ax2 = axes[0, 1]
    ax2.set_title('Cumulative Error Analysis', fontsize=12, fontweight='bold')
    
    for axis, color in [('roll', 'r'), ('pitch', 'g'), ('yaw', 'b')]:
        error_data = np.abs(df[f'err_{axis}'].values)
        sorted_errors = np.sort(error_data)
        cumulative = np.arange(1, len(sorted_errors) + 1) / len(sorted_errors)
        ax2.plot(sorted_errors, cumulative, color=color, label=f'{axis.capitalize()}', linewidth=2)
    
    ax2.set_xlabel('Absolute Error (deg)')
    ax2.set_ylabel('Cumulative Probability')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # Plot 3: Error statistics comparison
    ax3 = axes[1, 0]
    ax3.set_title('Error Statistics Comparison', fontsize=12, fontweight='bold')
    
    axes_names = ['roll', 'pitch', 'yaw']
    mae_values = [metrics[axis]['mae'] for axis in axes_names]
    rmse_values = [metrics[axis]['rmse'] for axis in axes_names]
    std_values = [metrics[axis]['std'] for axis in axes_names]
    
    x = np.arange(len(axes_names))
    width = 0.25
    
    ax3.bar(x - width, mae_values, width, label='MAE', alpha=0.8)
    ax3.bar(x, rmse_values, width, label='RMSE', alpha=0.8)
    ax3.bar(x + width, std_values, width, label='Std Dev', alpha=0.8)
    
    ax3.set_xlabel('Axis')
    ax3.set_ylabel('Error (deg)')
    ax3.set_xticks(x)
    ax3.set_xticklabels([axis.capitalize() for axis in axes_names])
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    # Plot 4: Settling time comparison
    ax4 = axes[1, 1]
    ax4.set_title('Settling Time Analysis', fontsize=12, fontweight='bold')
    
    settling_2pct = [metrics[axis]['settling_time_2pct'] for axis in axes_names]
    settling_5pct = [metrics[axis]['settling_time_5pct'] for axis in axes_names]
    
    # Replace None with 0 for plotting
    settling_2pct = [st if st is not None else 0 for st in settling_2pct]
    settling_5pct = [st if st is not None else 0 for st in settling_5pct]
    
    x = np.arange(len(axes_names))
    width = 0.35
    
    ax4.bar(x - width/2, settling_2pct, width, label='2% Tolerance', alpha=0.8)
    ax4.bar(x + width/2, settling_5pct, width, label='5% Tolerance', alpha=0.8)
    
    ax4.set_xlabel('Axis')
    ax4.set_ylabel('Settling Time (s)')
    ax4.set_xticks(x)
    ax4.set_xticklabels([axis.capitalize() for axis in axes_names])
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    
    plt.tight_layout()
    return fig


def create_integrity_plot(df, integrity_report):
    """Create data integrity visualization"""
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    fig.suptitle('Data Integrity Analysis', fontsize=16, fontweight='bold')
    
    # Plot 1: Data quality score gauge
    ax1 = axes[0, 0]
    ax1.set_title('Overall Data Quality Score', fontsize=12, fontweight='bold')
    
    quality_score = integrity_report['data_quality_score']
    
    # Create gauge visualization
    theta = np.linspace(0, np.pi, 100)
    r_outer = 1.0
    r_inner = 0.6
    
    # Background arc
    ax1.fill_between(theta, r_inner, r_outer, color='lightgray', alpha=0.3)
    
    # Quality score arc
    quality_theta = np.linspace(0, np.pi * quality_score / 100, 100)
    if quality_score >= 90:
        color = 'green'
    elif quality_score >= 75:
        color = 'yellow'
    elif quality_score >= 60:
        color = 'orange'
    else:
        color = 'red'
    
    ax1.fill_between(quality_theta, r_inner, r_outer, color=color, alpha=0.8)
    
    ax1.text(0, 0.8, f'{quality_score:.1f}%', ha='center', va='center', fontsize=24, fontweight='bold')
    ax1.text(0, 0.5, 'Data Quality', ha='center', va='center', fontsize=12)
    ax1.set_xlim(-1.2, 1.2)
    ax1.set_ylim(-0.2, 1.2)
    ax1.axis('off')
    
    # Plot 2: Missing values
    ax2 = axes[0, 1]
    ax2.set_title('Missing Values Analysis', fontsize=12, fontweight='bold')
    
    if integrity_report['missing_values']:
        cols = list(integrity_report['missing_values'].keys())
        counts = list(integrity_report['missing_values'].values())
        ax2.barh(cols, counts, color='red', alpha=0.7)
        ax2.set_xlabel('Missing Count')
    else:
        ax2.text(0.5, 0.5, '✓ No Missing Values', ha='center', va='center', 
                transform=ax2.transAxes, fontsize=16, color='green')
        ax2.set_xlim(0, 1)
        ax2.set_ylim(0, 1)
    
    ax2.grid(True, alpha=0.3)
    
    # Plot 3: Timestamp gaps
    ax3 = axes[1, 0]
    ax3.set_title('Timestamp Gap Analysis', fontsize=12, fontweight='bold')
    
    if integrity_report['timestamp_gaps']:
        gaps = integrity_report['timestamp_gaps']
        ax3.hist(gaps, bins=20, color='orange', alpha=0.7)
        ax3.set_xlabel('Gap Size (μs)')
        ax3.set_ylabel('Frequency')
        ax3.axvline(np.mean(gaps), color='red', linestyle='--', label=f'Mean: {np.mean(gaps):.0f} μs')
        ax3.legend()
    else:
        ax3.text(0.5, 0.5, '✓ No Significant Gaps', ha='center', va='center', 
                transform=ax3.transAxes, fontsize=16, color='green')
        ax3.set_xlim(0, 1)
        ax3.set_ylim(0, 1)
    
    ax3.grid(True, alpha=0.3)
    
    # Plot 4: Sensor anomalies
    ax4 = axes[1, 1]
    ax4.set_title('Sensor Anomaly Detection', fontsize=12, fontweight='bold')
    
    if integrity_report['sensor_anomalies']:
        anomalies = list(integrity_report['sensor_anomalies'].keys())
        ax4.barh(anomalies, [1] * len(anomalies), color='red', alpha=0.7)
        ax4.set_xlabel('Anomaly Detected')
    else:
        ax4.text(0.5, 0.5, '✓ No Sensor Anomalies', ha='center', va='center', 
                transform=ax4.transAxes, fontsize=16, color='green')
        ax4.set_xlim(0, 1)
        ax4.set_ylim(0, 1)
    
    ax4.grid(True, alpha=0.3)
    
    plt.tight_layout()
    return fig


def create_correlation_heatmap(df):
    """Create correlation heatmap for all sensors"""
    fig, ax = plt.subplots(figsize=(12, 10))
    
    # Select relevant columns for correlation
    sensor_cols = []
    for prefix in ['ctrl_', 'mon_']:
        for suffix in ['roll', 'pitch', 'yaw', 'acc_x', 'acc_y', 'acc_z', 'gyro_x', 'gyro_y', 'gyro_z']:
            col = f'{prefix}{suffix}'
            if col in df.columns:
                sensor_cols.append(col)
    
    if len(sensor_cols) > 0:
        correlation_matrix = df[sensor_cols].corr()
        
        # Create heatmap
        im = ax.imshow(correlation_matrix, cmap='coolwarm', aspect='auto', vmin=-1, vmax=1)
        
        # Set ticks and labels
        ax.set_xticks(range(len(sensor_cols)))
        ax.set_yticks(range(len(sensor_cols)))
        ax.set_xticklabels(sensor_cols, rotation=45, ha='right')
        ax.set_yticklabels(sensor_cols)
        
        # Add colorbar
        cbar = plt.colorbar(im, ax=ax)
        cbar.set_label('Correlation Coefficient')
        
        # Add correlation values as text
        for i in range(len(sensor_cols)):
            for j in range(len(sensor_cols)):
                text = ax.text(j, i, f'{correlation_matrix.iloc[i, j]:.2f}',
                             ha="center", va="center", color="black", fontsize=8)
        
        ax.set_title('Sensor Correlation Heatmap', fontsize=14, fontweight='bold')
    
    plt.tight_layout()
    return fig


def create_frequency_analysis_plot(df, metrics):
    """Create frequency domain analysis plot"""
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    fig.suptitle('Frequency Domain Analysis', fontsize=16, fontweight='bold')
    
    time_data = df['time_s'].values
    dt = time_data[1] - time_data[0] if len(time_data) > 1 else 1.0
    
    for idx, axis in enumerate(['roll', 'pitch', 'yaw']):
        ax = axes[idx // 2, idx % 2]
        
        error_data = df[f'err_{axis}'].values
        
        # Compute FFT
        fft = np.fft.fft(error_data)
        freqs = np.fft.fftfreq(len(error_data), d=dt)
        
        # Plot positive frequencies only
        pos_freqs = freqs[:len(freqs)//2]
        pos_fft = np.abs(fft[:len(fft)//2])
        
        ax.plot(pos_freqs, pos_fft, color=f'C{idx}', linewidth=1.5)
        ax.set_title(f'{axis.capitalize()} Error Spectrum', fontsize=12, fontweight='bold')
        ax.set_xlabel('Frequency (Hz)')
        ax.set_ylabel('Magnitude')
        ax.grid(True, alpha=0.3)
        ax.set_xlim(0, 10)  # Limit to 10 Hz for readability
        
        # Mark dominant frequency
        if metrics[axis]['dominant_frequency'] > 0:
            dom_freq = metrics[axis]['dominant_frequency']
            ax.axvline(x=dom_freq, color='red', linestyle='--', alpha=0.7, 
                      label=f"Dominant: {dom_freq:.2f} Hz")
            ax.legend()
    
    # Remove empty subplot if odd number of axes
    if len(['roll', 'pitch', 'yaw']) % 2 == 1:
        fig.delaxes(axes[1, 1])
    
    plt.tight_layout()
    return fig


def save_comprehensive_report(csv_path, metrics, df, integrity_report, correlation_metrics, report_dir, saved_plots):
    """Save comprehensive stability and integrity report to a text file"""
    report_path = os.path.join(report_dir, "comprehensive_stability_report.txt")
    
    try:
        with open(report_path, 'w') as f:
            f.write("=" * 70 + "\n")
            f.write("Comprehensive Stability and Integrity Report\n")
            f.write("=" * 70 + "\n\n")
            f.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"Data file: {os.path.basename(csv_path)}\n")
            f.write(f"Total samples: {len(df)}\n")
            f.write(f"Time span: {df['time_s'].iloc[-1]:.2f} seconds\n\n")
            
            # Report structure section
            f.write("=" * 70 + "\n")
            f.write("Report Structure\n")
            f.write("=" * 70 + "\n\n")
            f.write(f"Report Directory: {os.path.basename(report_dir)}\n")
            f.write("Contents:\n")
            f.write("  ├── comprehensive_stability_report.txt (this file)\n")
            f.write("  ├── plots/ (visualization directory)\n")
            if saved_plots:
                for plot_name, plot_path in saved_plots.items():
                    f.write(f"  │   ├── {os.path.basename(plot_path)}\n")
            f.write("  └── data/ (additional data files)\n\n")
            
            # Data Integrity Section
            f.write("=" * 70 + "\n")
            f.write("Data Integrity Validation\n")
            f.write("=" * 70 + "\n\n")
            f.write(f"Data Quality Score: {integrity_report['data_quality_score']:.1f}/100\n")
            f.write(f"Total Samples: {integrity_report['total_samples']}\n\n")
            
            if integrity_report['missing_values']:
                f.write("Missing Values:\n")
                for col, count in integrity_report['missing_values'].items():
                    f.write(f"  {col}: {count}\n")
            else:
                f.write("✓ No missing values detected\n\n")
            
            if integrity_report['out_of_range_values']:
                f.write("Out-of-Range Values:\n")
                for col, count in integrity_report['out_of_range_values'].items():
                    f.write(f"  {col}: {count}\n")
            else:
                f.write("✓ All values within expected ranges\n\n")
            
            if integrity_report['timestamp_gaps']:
                f.write(f"Timestamp Gaps (packet loss): {len(integrity_report['timestamp_gaps'])}\n")
                f.write(f"  Average gap size: {np.mean(integrity_report['timestamp_gaps']):.0f} μs\n\n")
            else:
                f.write("✓ No significant timestamp gaps detected\n\n")
            
            if integrity_report['sensor_anomalies']:
                f.write("Sensor Anomalies:\n")
                for anomaly, flag in integrity_report['sensor_anomalies'].items():
                    f.write(f"  {anomaly}: {flag}\n")
            else:
                f.write("✓ No sensor anomalies detected\n\n")
            
            # Advanced Stability Metrics Section
            f.write("=" * 70 + "\n")
            f.write("Advanced Stability Metrics\n")
            f.write("=" * 70 + "\n\n")
            
            for axis in ['roll', 'pitch', 'yaw']:
                f.write(f"{axis.upper()} Axis:\n")
                f.write("  " + "-" * 50 + "\n")
                
                m = metrics[axis]
                
                f.write("  Basic Metrics:\n")
                f.write(f"    MAE: {m['mae']:.6f}°\n")
                f.write(f"    RMSE: {m['rmse']:.6f}°\n")
                f.write(f"    Std Dev: {m['std']:.6f}°\n")
                f.write(f"    Peak-to-Peak: {m['peak_to_peak']:.6f}°\n")
                f.write(f"    95th Percentile: {m['percentile_95']:.6f}°\n\n")
                
                f.write("  Advanced Metrics:\n")
                f.write(f"    Signal-to-Noise Ratio: {m['snr_db']:.2f} dB\n")
                f.write(f"    Stability Index: {m['stability_index']:.6f}\n")
                f.write(f"    Skewness: {m['skewness']:.6f}\n")
                f.write(f"    Kurtosis: {m['kurtosis']:.6f}\n")
                f.write(f"    Drift Rate: {m['drift_rate']:.9f}°/s\n")
                f.write(f"    Dominant Frequency: {m['dominant_frequency']:.6f} Hz\n\n")
                
                f.write("  Time-Based Metrics:\n")
                if m['convergence_time'] is not None:
                    f.write(f"    Convergence Time: {m['convergence_time']:.6f} s\n")
                else:
                    f.write(f"    Convergence Time: Not converged\n")
                
                if m['settling_time_2pct'] is not None:
                    f.write(f"    Settling Time (2%): {m['settling_time_2pct']:.6f} s\n")
                else:
                    f.write(f"    Settling Time (2%): Not settled\n")
                
                if m['settling_time_5pct'] is not None:
                    f.write(f"    Settling Time (5%): {m['settling_time_5pct']:.6f} s\n")
                else:
                    f.write(f"    Settling Time (5%): Not settled\n")
                
                f.write("\n")
            
            # Cross-correlation metrics
            if correlation_metrics:
                f.write("=" * 70 + "\n")
                f.write("Cross-Correlation Metrics (Control vs Monitor)\n")
                f.write("=" * 70 + "\n\n")
                
                f.write("Attitude Correlation:\n")
                for axis in ['roll', 'pitch', 'yaw']:
                    key = f'attitude_{axis}_correlation'
                    if key in correlation_metrics:
                        f.write(f"  {axis.capitalize()}: {correlation_metrics[key]:.6f}\n")
                
                f.write("\nSensor Correlation:\n")
                for sensor in ['acc', 'gyro']:
                    for axis in ['x', 'y', 'z']:
                        key = f'{sensor}_{axis}_correlation'
                        if key in correlation_metrics:
                            f.write(f"  {sensor.upper()} {axis.upper()}: {correlation_metrics[key]:.6f}\n")
                f.write("\n")
            
            # Additional Statistics Section
            f.write("=" * 70 + "\n")
            f.write("Additional Statistics\n")
            f.write("=" * 70 + "\n\n")
            
            f.write("Attitude Error Statistics:\n")
            for axis in ['roll', 'pitch', 'yaw']:
                err_col = f'err_{axis}'
                f.write(f"  {axis.capitalize():5s}: ")
                f.write(f"mean={df[err_col].mean():.6f}°, ")
                f.write(f"std={df[err_col].std():.6f}°, ")
                f.write(f"max_abs={df[err_col].abs().max():.6f}°\n")
            
            f.write("\nData Quality Assessment:\n")
            f.write(f"  Overall Quality Score: {integrity_report['data_quality_score']:.1f}/100\n")
            
            # Quality classification
            if integrity_report['data_quality_score'] >= 90:
                quality_level = "Excellent"
            elif integrity_report['data_quality_score'] >= 75:
                quality_level = "Good"
            elif integrity_report['data_quality_score'] >= 60:
                quality_level = "Fair"
            else:
                quality_level = "Poor"
            
            f.write(f"  Quality Level: {quality_level}\n")
            
            # Stability assessment
            avg_mae = np.mean([metrics[axis]['mae'] for axis in ['roll', 'pitch', 'yaw']])
            avg_rmse = np.mean([metrics[axis]['rmse'] for axis in ['roll', 'pitch', 'yaw']])
            
            f.write("\nStability Assessment:\n")
            f.write(f"  Average MAE: {avg_mae:.6f}°\n")
            f.write(f"  Average RMSE: {avg_rmse:.6f}°\n")
            
            if avg_mae < 0.5 and avg_rmse < 1.0:
                stability_level = "Excellent"
            elif avg_mae < 1.0 and avg_rmse < 2.0:
                stability_level = "Good"
            elif avg_mae < 2.0 and avg_rmse < 4.0:
                stability_level = "Fair"
            else:
                stability_level = "Poor"
            
            f.write(f"  Stability Level: {stability_level}\n")
            
            # Plot descriptions
            f.write("\n" + "=" * 70 + "\n")
            f.write("Generated Visualizations\n")
            f.write("=" * 70 + "\n\n")
            
            plot_descriptions = {
                'main': 'Main analysis plot showing attitude comparison, errors, accelerometer and gyroscope data',
                'error_analysis': 'Detailed error analysis including distributions, cumulative errors, statistics comparison, and settling times',
                'integrity': 'Data integrity visualization with quality gauge, missing values, timestamp gaps, and sensor anomalies',
                'correlation': 'Correlation heatmap showing relationships between all sensor channels',
                'frequency': 'Frequency domain analysis showing error spectra and dominant frequencies'
            }
            
            if saved_plots:
                for plot_name, plot_path in saved_plots.items():
                    description = plot_descriptions.get(plot_name, 'Analysis plot')
                    f.write(f"{os.path.basename(plot_path)}:\n")
                    f.write(f"  {description}\n\n")
        
        print(f"Comprehensive stability report saved to: {report_path}")
    except Exception as e:
        print(f"Error saving comprehensive report: {e}")


def main():
    """Main analysis function with comprehensive validation and metrics"""
    print("=" * 70)
    print("FCU IMU Telemetry - Comprehensive Log Analysis Tool")
    print("=" * 70)
    
    # Select log file
    csv_path = select_log_file()
    
    if csv_path is None:
        print("No file selected. Exiting.")
        return
    
    # Load data
    df = load_csv_data(csv_path)
    
    if df is None or len(df) == 0:
        print("No data to analyze.")
        return
    
    # Create timestamped report directory
    report_dir, plots_dir, data_dir = create_report_directory(csv_path)
    print(f"\nReport directory created: {os.path.basename(report_dir)}")
    
    print("\n" + "=" * 70)
    print("Starting Comprehensive Analysis...")
    print("=" * 70)
    
    # 1. Data Integrity Validation
    print("\n1. Performing data integrity validation...")
    integrity_report = validate_data_integrity(df)
    print_data_integrity_report(integrity_report)
    
    # 2. Basic Statistics
    print("\n2. Generating basic statistics...")
    print_data_statistics(df)
    
    # 3. Advanced Stability Metrics
    print("\n3. Calculating advanced stability metrics...")
    stability_metrics = calculate_stability_metrics(df)
    
    # 4. Cross-correlation Analysis
    print("\n4. Performing cross-correlation analysis...")
    correlation_metrics = calculate_cross_correlation_metrics(df)
    
    # 5. Print Advanced Metrics
    print("\n5. Advanced stability analysis results:")
    advanced_metrics = print_advanced_stability_metrics(stability_metrics, correlation_metrics)
    
    # 6. Create Enhanced Plots
    print("\n6. Generating enhanced analysis plots...")
    fig = create_analysis_plots(df)
    
    if fig is not None:
        # Save enhanced plots to timestamped directory
        print("\n6a. Saving enhanced plots to report directory...")
        saved_plots = save_enhanced_plots(fig, df, plots_dir, stability_metrics, integrity_report)
        
        # Save comprehensive report
        print("\n6b. Generating comprehensive report...")
        save_comprehensive_report(csv_path, stability_metrics, df, integrity_report, correlation_metrics, report_dir, saved_plots)
        
        # Save copy of original data for reference
        data_copy_path = os.path.join(data_dir, "original_data.csv")
        df.to_csv(data_copy_path, index=False)
        print(f"\n6c. Original data saved to: {data_copy_path}")
        
        # Show main plot
        print("\nDisplaying main analysis plot. Close the window to exit.")
        plt.show()
    else:
        print("Failed to create plots.")
    
    # 7. Summary Assessment
    print("\n" + "=" * 70)
    print("ANALYSIS SUMMARY")
    print("=" * 70)
    
    # Data quality summary
    quality_score = integrity_report['data_quality_score']
    if quality_score >= 90:
        quality_status = "✓ EXCELLENT"
    elif quality_score >= 75:
        quality_status = "✓ GOOD"
    elif quality_score >= 60:
        quality_status = "⚠ FAIR"
    else:
        quality_status = "✗ POOR"
    
    print(f"Data Quality: {quality_status} ({quality_score:.1f}/100)")
    
    # Stability summary
    avg_mae = np.mean([stability_metrics[axis]['mae'] for axis in ['roll', 'pitch', 'yaw']])
    avg_rmse = np.mean([stability_metrics[axis]['rmse'] for axis in ['roll', 'pitch', 'yaw']])
    
    if avg_mae < 0.5 and avg_rmse < 1.0:
        stability_status = "✓ EXCELLENT"
    elif avg_mae < 1.0 and avg_rmse < 2.0:
        stability_status = "✓ GOOD"
    elif avg_mae < 2.0 and avg_rmse < 4.0:
        stability_status = "⚠ FAIR"
    else:
        stability_status = "✗ POOR"
    
    print(f"Stability Performance: {stability_status}")
    print(f"  Average MAE: {avg_mae:.4f}°")
    print(f"  Average RMSE: {avg_rmse:.4f}°")
    
    # Cross-correlation summary
    if correlation_metrics:
        avg_attitude_corr = np.mean([correlation_metrics.get(f'attitude_{axis}_correlation', 0) 
                                   for axis in ['roll', 'pitch', 'yaw']])
        print(f"IMU Correlation: {avg_attitude_corr:.4f} (Control vs Monitor)")
    
    print("\nGenerated Report Structure:")
    print(f"  📁 {os.path.basename(report_dir)}/")
    print(f"    ├── 📄 comprehensive_stability_report.txt")
    print(f"    ├── 📁 plots/")
    if saved_plots:
        for plot_name, plot_path in saved_plots.items():
            print(f"    │   ├── 🖼️ {os.path.basename(plot_path)}")
    print(f"    └── 📁 data/")
    print(f"        ├── 📄 original_data.csv")
    
    print("\n" + "=" * 70)
    print("Analysis complete!")
    print(f"Report location: {report_dir}")
    print("=" * 70)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nAnalysis cancelled by user.")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
