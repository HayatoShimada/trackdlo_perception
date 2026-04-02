# Spec A: コード品質改善 設計ドキュメント

## 概要

trackdlo_perceptionをROS2コミュニティ公開に適した品質にするため、copyrightヘッダ追加、include guard修正、ament_lint完全対応を行う。

## 1. Copyrightヘッダ追加

### 対象

- C++ 18ファイル（.hpp, .cpp, .cu, .cuh）
- Python 25ファイル（.py、launch.py含む）

### フォーマット

C++（ファイル先頭、`#pragma once`の直前）:
```cpp
// Copyright 2026 Hayato Shimada
// SPDX-License-Identifier: BSD-3-Clause
```

Python（ファイル先頭、shebangの直後）:
```python
# Copyright 2026 Hayato Shimada
# SPDX-License-Identifier: BSD-3-Clause
```

## 2. Include Guard修正

8ヘッダファイル全て、ROS2標準パターン `TRACKDLO_CORE__<FILENAME>_HPP_` に統一。

| ファイル | 現在 | 修正後 |
|---|---|---|
| trackdlo.hpp | `TRACKDLO_PERCEPTION__TRACKDLO_HPP_` | `TRACKDLO_CORE__TRACKDLO_HPP_` |
| utils.hpp | `TRACKDLO_PERCEPTION__UTILS_HPP_` | `TRACKDLO_CORE__UTILS_HPP_` |
| evaluator.hpp | `TRACKDLO_PERCEPTION__EVALUATOR_HPP_` | `TRACKDLO_CORE__EVALUATOR_HPP_` |
| image_preprocessor.hpp | `TRACKDLO_PERCEPTION_IMAGE_PREPROCESSOR_HPP` | `TRACKDLO_CORE__IMAGE_PREPROCESSOR_HPP_` |
| pipeline_manager.hpp | `TRACKDLO_PIPELINE_MANAGER_HPP` | `TRACKDLO_CORE__PIPELINE_MANAGER_HPP_` |
| visibility_checker.hpp | `TRACKDLO_PERCEPTION_VISIBILITY_CHECKER_HPP` | `TRACKDLO_CORE__VISIBILITY_CHECKER_HPP_` |
| visualizer.hpp | `TRACKDLO_VISUALIZER_HPP` | `TRACKDLO_CORE__VISUALIZER_HPP_` |
| pointcloud_cuda.cuh | `POINTCLOUD_CUDA_CUH` | `TRACKDLO_CORE__POINTCLOUD_CUDA_CUH_` |

`#pragma once`と`#ifndef`ガードを併用。

## 3. ament_lint完全対応

### uncrustify

ROS2デフォルトのuncrustify設定で全C++ファイルを`ament_uncrustify --reformat`で自動フォーマット。コードスタイルがAllmanから1TBSに変わるが、機能変更なし。

### cpplint

Google C++ Style準拠。uncrustifyと競合する部分はフィルタで除外。

### flake8

`max-line-length=150`、`ignore=E501,W503,E741`を`.flake8`設定ファイルで指定。

### pep257

公開クラス・関数にdocstringを追加。

### copyright

SPDX形式ヘッダをament_copyrightが認識。

### CMakeLists.txt

全3パッケージ（trackdlo_core, trackdlo_bringup, trackdlo_msgs）のCMakeLists.txtからAMENT_LINT_AUTO_EXCLUDEリストを削除。

```cmake
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
```

## 実行順序

1. copyrightヘッダを全43ファイルに追加
2. include guardを8ファイルで修正
3. `ament_uncrustify --reformat`でC++フォーマット
4. flake8/pep257エラーをPythonファイルで修正
5. CMakeLists.txtからexcludeリスト削除
6. `colcon build && colcon test`で全テスト通過を確認
7. コミット

## 成功基準

`colcon test --return-code-on-test-failure`が全パッケージで0エラー、0失敗で通過すること。
