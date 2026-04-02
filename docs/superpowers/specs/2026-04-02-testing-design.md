# Spec B: テスト追加 設計ドキュメント

## 概要

trackdlo_coreのコアアルゴリズムにGTestユニットテストを追加し、trackdlo_bringupにlaunchテストを追加する。

## C++ユニットテスト (GTest)

### 1. test_trackdlo.cpp — CPD-LLEアルゴリズム
- 初期化テスト: ノード数、パラメータ設定
- cpd_lle収束テスト: 既知の点群に対するノード移動
- エッジケース: 空の点群、極小点群

### 2. test_image_preprocessor.cpp — 画像前処理
- HSVマスク生成: 青い画像→全白マスク、黒い画像→全黒マスク
- 外部マスクモード: マスクパススルー

### 3. test_visibility_checker.cpp — 可視性判定
- 全ノード可視→全ノード返却
- 一部ノードoccluded→除外

## Launchテスト

### test_launch.py — Launch fileパース検証
- trackdlo.launch.pyが構文エラーなくロードできるか
- launch引数の定義検証

## ファイル配置

```
trackdlo_core/test/
├── test_trackdlo.cpp
├── test_image_preprocessor.cpp
└── test_visibility_checker.cpp

trackdlo_bringup/test/
└── test_launch.py
```

## 成功基準

`colcon test --return-code-on-test-failure`が全パッケージで0エラー、0失敗。
