# Phase 2: bloom release + CI/CD 設計ドキュメント

## 概要

GitHub Actions CI/CDパイプラインを構築し、bloom-releaseでリリースリポジトリを作成し、GitHub Pagesでaptリポジトリを公開する。

## 構成要素

1. GitHub Actions CI（ビルド + テスト + lint）
2. bloom-release（リリースリポジトリ作成）
3. GitHub Pages apt（debianパッケージ配布）

## 1. GitHub Actions CI

### ワークフロー: `.github/workflows/build.yml`

- トリガー: push to master, PR to master
- マトリクス: Humble × amd64, Jazzy × amd64
- コンテナ: `ros:<distro>-ros-base`
- ステップ:
  1. checkout
  2. `rosdep install` で依存解決
  3. `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DUSE_CUDA=OFF`
  4. `colcon test --return-code-on-test-failure`
  5. `flake8` lint（trackdlo_core, trackdlo_utils, trackdlo_segmentation）
- CUDAはCI環境にないため`-DUSE_CUDA=OFF`で無効化

## 2. bloom-release

### 実行コマンド

```bash
bloom-release --rosdistro humble --track humble trackdlo_perception --new-track
```

- `HayatoShimada/trackdlo_perception-release` リポジトリが自動作成される
- 全5パッケージのdebian制御ファイルが生成される
- v2.0.0タグからリリース

### 前提条件

- GitHub認証済み
- git user.name / user.email 設定済み
- masterブランチのv2.0.0タグがpush済み

## 3. GitHub Pages aptリポジトリ

### ワークフロー: `.github/workflows/release-deb.yml`

- トリガー: `v*` タグpush時
- コンテナ: `ros:humble-ros-base`
- ステップ:
  1. checkout
  2. rosdep install
  3. colcon buildでワークスペースビルド
  4. bloom-generateでdebian制御ファイル生成（各パッケージ個別）
  5. dpkg-buildpackageでdebパッケージ作成
  6. apt-ftparchiveでPackages/Releaseファイル生成
  7. gh-pagesブランチにdebファイル + メタデータをコミット
  8. GitHub Pagesで公開

### ディレクトリ構造（gh-pages）

```
humble/
├── ros-humble-trackdlo-core_2.0.0-0_amd64.deb
├── ros-humble-trackdlo-segmentation_2.0.0-0_amd64.deb
├── ros-humble-trackdlo-msgs_2.0.0-0_amd64.deb
├── ros-humble-trackdlo-bringup_2.0.0-0_amd64.deb
├── ros-humble-trackdlo-utils_2.0.0-0_amd64.deb
├── Packages
├── Packages.gz
└── Release
```

### ユーザーのインストール手順

```bash
echo "deb [trusted=yes] https://hayatoshimada.github.io/trackdlo_perception/humble /" \
  | sudo tee /etc/apt/sources.list.d/trackdlo.list
sudo apt update
sudo apt install ros-humble-trackdlo-core ros-humble-trackdlo-segmentation
```

### 注意点

- GPG署名なし（`[trusted=yes]`で回避）
- Humbleのみ最初に対応
- Jazzyは後でマトリクス追加可能
