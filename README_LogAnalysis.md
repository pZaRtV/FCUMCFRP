# Log Analysis Tool for FCU IMU Telemetry

Post-processing analysis tool for saved IMU telemetry CSV log files.

## Enhanced Features (Version 2.0)

This enhanced version includes comprehensive **event detection analysis and disturbance monitoring**:

- **Advanced Stability Metrics**: 15 different stability validation metrics
- **Event Pattern Analysis**: Command vs. disturbance event analysis
- **Data Integrity Validation**: UDP packet structure and quality assessment
- **Cross-Correlation Analysis**: Control vs. Monitor IMU correlation studies
- **Comprehensive Reporting**: Timestamped report directories with multiple visualizations
- **Disturbance Statistics**: Wind gust and turbulence response analysis
- **Research-grade Analysis**: Perfect for flight control system validation

## Key Features

### Core Functionality
- **Interactive Log Selection**: Browse and select from timestamped log directories
- **Comprehensive Statistics**: Detailed statistical analysis of all sensor data
- **Enhanced Stability Validation**: 15 different stability metrics
- **Visual Analysis**: Multiple plot types for comprehensive analysis
- **Auto-Save**: Automatically saves plots and comprehensive reports

### Advanced Stability Metrics
- **Basic Metrics**: MAE, RMSE, Standard Deviation, Settling Time
- **Signal-to-Noise Ratio (SNR)**: Quality of control signal in dB
- **Peak-to-Peak Error**: Maximum error range analysis
- **Error Distribution**: Skewness and kurtosis for error pattern analysis
- **95th Percentile Error**: Robust error assessment
- **Stability Index**: Ratio of std to MAE for stability assessment
- **Convergence Time**: Time to reach steady-state performance
- **Drift Rate**: Linear trend detection in error signals
- **Frequency Analysis**: Dominant frequency identification

### Event Detection Analysis
- **Command Event Analysis**: Pattern and frequency of command inputs
- **Disturbance Event Statistics**: Count, magnitude, and timing of disturbances
- **Event Correlation**: Relationship between commands and disturbances
- **Response Time Analysis**: Control system response to events
- **Event Timeline Visualization**: Temporal event pattern analysis

### Data Integrity Validation
- **UDP Packet Validation**: Structure and size verification
- **Missing Value Detection**: Data completeness assessment
- **Out-of-Range Checking**: Sensor data validation
- **Timestamp Gap Analysis**: Packet loss detection
- **Sensor Anomaly Detection**: Stuck values and excessive noise
- **Quality Scoring**: Overall data quality assessment (0-100)

### Cross-Correlation Analysis
- **Attitude Correlation**: Control vs. Monitor IMU attitude consistency
- **Raw Sensor Correlation**: Accelerometer and gyroscope correlation
- **Sensor Fusion Validation**: Madgwick filter performance assessment
- **Calibration Verification**: Sensor calibration consistency

## Requirements

Install dependencies:
```bash
pip install -r requirements.txt
```

Required packages:
- `numpy` >= 1.21.0
- `matplotlib` >= 3.5.0
- `pandas` >= 1.3.0
- `scipy` >= 1.7.0 (for advanced analysis)
- `seaborn` >= 0.11.0 (for enhanced visualizations)

## Usage

1. **Run enhanced analysis tool**:
   ```bash
   python FlightlogAnalysis.py
   ```

2. **Select a log file**:
   - The tool will list all available log files in `data_logs/`
   - Each entry shows: timestamp, number of rows, file size
   - Enter the number corresponding to the log file you want to analyze
   - Or press 'q' to quit

3. **Comprehensive Analysis Process**:
   - Data integrity validation and quality assessment
   - Advanced stability metrics calculation
   - Event pattern and disturbance analysis
   - Cross-correlation and frequency analysis
   - Multiple visualization generation
   - Comprehensive report creation

4. **View Results**:
   - Real-time console output of analysis progress
   - Interactive plots displayed (5 different visualizations)
   - Comprehensive text report with all metrics
   - Quality and stability level classification

## Output Structure

### Timestamped Report Directory
```bash
report_YYYYMMDD_HHMMSS/
├── comprehensive_stability_report.txt    # Detailed analysis report
├── plots/                                 # Visualization directory
│   ├── main_analysis.png                 # Primary 4-panel analysis
│   ├── error_analysis.png                # Detailed error analysis
│   ├── data_integrity.png                 # Data quality visualization
│   ├── correlation_heatmap.png           # Cross-correlation matrix
│   └── frequency_analysis.png            # Frequency domain analysis
└── data/
    └── original_data.csv                  # Copy of analyzed data
```

### Console Statistics

The tool prints comprehensive analysis including:
- **Data Quality Assessment**: Overall quality score and validation results
- **Advanced Stability Metrics**: All 15 stability metrics with detailed breakdowns
- **Event Analysis**: Command and disturbance event statistics
- **Cross-Correlation Results**: IMU correlation and consistency metrics
- **Frequency Analysis**: Dominant frequencies and spectral characteristics
- **Quality Classification**: Automated quality and stability level assessment

### Enhanced Plot Types

1. **Main Analysis Plot**: Traditional 4-panel visualization
   - Attitude comparison (Control vs Monitor)
   - Attitude error analysis with statistics
   - Accelerometer data comparison
   - Gyroscope data comparison

2. **Error Analysis Plot**: Detailed error visualization
   - Error distribution histograms
   - Cumulative error analysis
   - Statistics comparison (MAE, RMSE, Std Dev)
   - Settling time analysis

3. **Data Integrity Plot**: Quality assessment visualization
   - Data quality gauge
   - Missing value analysis
   - Timestamp gap visualization
   - Sensor anomaly detection

4. **Correlation Heatmap**: Cross-correlation matrix
   - All sensor channel correlations
   - Control vs. Monitor IMU relationships
   - Raw sensor vs. attitude correlations

5. **Frequency Analysis Plot**: Spectral analysis
   - Error signal spectra
   - Dominant frequency identification
   - SNR analysis
   - Frequency response characteristics

### Comprehensive Report

The `comprehensive_stability_report.txt` includes:
- **Report Structure**: File organization and contents
- **Data Integrity Validation**: Complete quality assessment
- **Advanced Stability Metrics**: Detailed metric breakdowns
- **Cross-Correlation Analysis**: Correlation results and interpretation
- **Event Analysis**: Command and disturbance statistics
- **Quality Assessment**: Automated classification and recommendations
- **Visualization Descriptions**: Explanation of all generated plots

## Advanced Metrics Explained

### Stability Metrics
- **MAE (Mean Absolute Error)**: Average magnitude of errors
- **RMSE (Root Mean Square Error)**: Penalizes larger errors more heavily
- **SNR (Signal-to-Noise Ratio)**: Quality of control signal in dB
- **Peak-to-Peak Error**: Maximum error range
- **Skewness**: Error distribution asymmetry
- **Kurtosis**: Error distribution tail weight
- **95th Percentile**: Robust error assessment
- **Stability Index**: Ratio of std to MAE
- **Convergence Time**: Time to reach steady-state
- **Drift Rate**: Linear trend in error
- **Dominant Frequency**: Primary oscillation frequency

### Event Analysis
- **Command Events**: Pilot input detection and analysis
- **Disturbance Events**: Uncommanded attitude changes
- **Event Frequency**: Timing and pattern analysis
- **Response Time**: Control system reaction time
- **Event Correlation**: Relationship between events

### Data Quality
- **Quality Score**: Overall data quality (0-100)
- **Completeness**: Missing value assessment
- **Consistency**: Data range validation
- **Reliability**: Timestamp and sensor analysis

## Research Applications

### Flight Control System Validation
- **Disturbance Rejection**: Test response to wind gusts
- **Algorithm Tuning**: Optimize control parameters
- **Robustness Testing**: Evaluate turbulent conditions
- **Performance Benchmarking**: Compare different configurations

### Disturbance Analysis
- **Wind Gust Characterization**: Study disturbance patterns
- **Turbulence Response**: Analyze control behavior
- **External Force Detection**: Identify impacts
- **Environmental Assessment**: Evaluate flight conditions

### Sensor Fusion Validation
- **IMU Consistency**: Control vs. Monitor comparison
- **Filter Performance**: Madgwick filter assessment
- **Calibration Verification**: Sensor accuracy validation
- **Cross-Correlation**: Sensor relationship analysis

## Example Output

```
======================================================================
FCU IMU Telemetry - Comprehensive Log Analysis Tool
======================================================================

======================================================================
Available Log Files:
======================================================================
  [ 1] 2026-01-22 06:48:15 |   5234 rows | 0.65 MB
  [ 2] 2026-01-22 05:30:45 |   8678 rows | 1.08 MB
  [ 3] 2026-01-22 04:15:10 |  12012 rows | 1.50 MB
======================================================================

Select log file [1-3] (or 'q' to quit): 1

Report directory created: report_20250122_064815

======================================================================
Starting Comprehensive Analysis...
======================================================================

1. Performing data integrity validation...
Data Quality Score: 94.2/100
 No missing values detected
 All values within expected ranges
 No significant timestamp gaps detected
 No sensor anomalies detected

2. Generating basic statistics...

3. Calculating advanced stability metrics...

4. Performing cross-correlation analysis...

5. Advanced stability analysis results:
ROLL Axis:
  Basic Metrics:
    MAE: 0.8234°
    RMSE: 1.2456°
    Std Dev: 0.9345°
  Advanced Metrics:
    Signal-to-Noise Ratio: 15.23 dB
    Stability Index: 1.134
    Drift Rate: 0.000123°/s
    Dominant Frequency: 2.34 Hz

...

======================================================================
ANALYSIS SUMMARY
======================================================================
Data Quality:  EXCELLENT (94.2/100)
Stability Performance:  GOOD
  Average MAE: 0.8234°
  Average RMSE: 1.2456°
IMU Correlation: 0.9876 (Control vs. Monitor)

Generated Report Structure:
  report_20250122_064815/
    comprehensive_stability_report.txt
    plots/
      main_analysis.png
      error_analysis.png
      data_integrity.png
      correlation_heatmap.png
      frequency_analysis.png
    data/
      original_data.csv

======================================================================
Analysis complete!
Report location: report_20250122_064815
======================================================================
```

## Notes

- Log files are sorted by timestamp (newest first)
- Only directories containing `imu_data.csv` are listed
- Enhanced CSV files with `event_flags` column are automatically detected
- All plots use consistent color schemes for professional presentation
- Reports include quality and stability level classifications
- Event analysis requires enhanced CSV format (version 1.1+)
- Frequency analysis uses FFT for spectral characterization
- Cross-correlation analysis validates sensor fusion performance

## Version History

- **v1.0**: Basic stability metrics and 4-panel plotting
- **v2.0**: Enhanced with event detection analysis, advanced metrics, and comprehensive reporting
