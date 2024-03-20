# hdl_global_localization

## Requirements
***hdl_global_localization*** requires the following libraries:
- PCL
- OpenCV
- OpenMP
- Teaser++ [Optional]

## Services

- ***/hdl_global_localization/set_engine*** (hdl_global_localization::SetGlobalLocalizationEngine)
  - Available global localization engines: BBS, FPFH_RANSAC, FPFH_TEASER
- ***/hdl_global_localization/set_global_map*** (hdl_global_localization::SetGlobalMap)
- ***/hdl_global_localization/query*** (hdl_global_localization::QueryGlobalLocalization)


## Algorithms

- 2D Grid Map-based Branch-and-Bound Search
  - Real-time loop closure in 2D LIDAR SLAM, ICRA, 2016
- FPFH + RANSAC (based on pcl::SampleConsensusPrerejective)
  - Fast Point Feature Histograms (FPFH) for 3D registration, ICRA, 2009
  - Pose Estimation using Local Structure-Specific Shape and Appearance Context, ICRA, 2013
- FPFH + Teaser++
  - TEASER: Fast and Certifiable Point Cloud Registration, T-RO, 2020