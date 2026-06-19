# STM32 ビルドエグゼキューター - セットアップガイド

このドキュメントは、`stm32_build_executor` サブエージェントを他のプロジェクトやPCで使用するための手順です。

## ファイル構成

```
.claude/agents/stm32_build_executor/
├── agent.md              # サブエージェント定義（メイン）
├── build.config.json     # ビルド設定（環境ごとに編集）
└── SETUP.md             # このファイル
```

---

## 初回セットアップ（現在のプロジェクト）

このプロジェクト（PFLIGHT_26）では、すでに設定済みです。

```bash
# 確認方法
cat .claude/agents/stm32_build_executor/build.config.json
```

---

## 他のPCでの セットアップ手順

### ステップ1: 設定ファイルを確認

```bash
cd <プロジェクトルート>/.claude/agents/stm32_build_executor
cat build.config.json
```

### ステップ2: パスを修正

`build.config.json` をテキストエディタで開き、以下を環境に合わせて修正：

```json
{
  "project_name": "PFLIGHT_26",                  // プロジェクト名（そのままでOK）
  "project_root": "d:/GitLocal/STM32CubeIDE/PFLIGHT_26",  // ← 修正
  "stm32_cubide_path": "D:\\ST\\STM32CubeIDE_1.19.0",     // ← 修正
  "make_tool_path": "...",                       // ← 修正
  "gnu_tools_path": "...",                       // ← 修正
  "debug_directory": "d:/GitLocal/STM32CubeIDE/PFLIGHT_26/Debug"  // ← 修正
}
```

#### パスの確認方法

**STM32CubeIDE のインストールパス：**
```bash
# Windows
echo $PROGRAMFILES\ST\STM32CubeIDE_*

# または IDE から確認
Help → About STM32CubeIDE → Installation Details
```

**Make ツール パス：**
```bash
# IDE の plugins フォルダから
<STM32CubeIDE>/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.make.win32_*/tools/bin
```

**GNU ツール パス：**
```bash
# IDE の plugins フォルダから
<STM32CubeIDE>/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.*/tools/bin
```

### ステップ3: ビルド実行

Claude Code から以下のように呼び出す：

```
プロジェクトをビルドしてください
```

サブエージェントが `build.config.json` から設定を読み込み、ビルドを実行します。

---

## 設定ファイル詳細

### build.config.json の各フィールド

| フィールド | 説明 | 例 |
|-----------|------|-----|
| `project_name` | プロジェクト名 | `PFLIGHT_26` |
| `project_root` | プロジェクトルート | `d:/GitLocal/STM32CubeIDE/PFLIGHT_26` |
| `stm32_cubide_path` | STM32CubeIDEインストール先 | `D:\\ST\\STM32CubeIDE_1.19.0` |
| `make_tool_path` | Make ツール | `D:\\ST\\...\\make.win32_2.2.0.202409170845\\tools\\bin` |
| `gnu_tools_path` | GNU ツールチェーン | `D:\\ST\\...\\gnu-tools-for-stm32.13.3.rel1.win32_1.0.0.202411081344\\tools\\bin` |
| `debug_directory` | Debug フォルダ | `d:/GitLocal/STM32CubeIDE/PFLIGHT_26/Debug` |

### build_options の設定

```json
"build_options": {
  "clean_before_build": false,  // true: make clean を実行
  "parallel_jobs": "auto",      // "auto" または 数字（4, 8, 16等）
  "verbose": false              // true: 詳細出力
}
```

---

## トラブルシューティング

### エラー: "No such file or directory"

**原因：** パスが正しく設定されていない

**対処：**
1. `build.config.json` で指定したパスが実際に存在するか確認
2. パスのバックスラッシュが正しく エスケープされているか確認
3. ファイル名に タイプミスがないか確認

```bash
# Linux/Mac
ls -la /path/to/file

# Windows (Git Bash)
ls -la "D:/path/to/file"
```

### エラー: "arm-none-eabi-size Error -1"

**原因：** GNU ツール パスが正しく設定されていない

**対処：**
1. `gnu_tools_path` が正確か確認
2. IDE の plugins フォルダで正しいバージョンを確認
3. パスが存在するか確認

```bash
ls -la "<gnu_tools_path>/arm-none-eabi-size"
```

### エラー: "make: command not found"

**原因：** Make ツール パスが正しく設定されていない

**対処：**
1. `make_tool_path` が正確か確認
2. ファイルが `make.exe` という名前か確認

```bash
ls -la "<make_tool_path>/make.exe"
```

---

## 複数プロジェクトの管理

異なるプロジェクトで使用する場合：

1. **各プロジェクトに同じ agents フォルダを配置**
   ```
   Project_A/.claude/agents/stm32_build_executor/
   Project_B/.claude/agents/stm32_build_executor/
   ```

2. **各プロジェクトで build.config.json を修正**
   ```bash
   # Project_A
   nano Project_A/.claude/agents/stm32_build_executor/build.config.json

   # Project_B
   nano Project_B/.claude/agents/stm32_build_executor/build.config.json
   ```

3. **各プロジェクトで独立してビルド可能**

---

## 配布時のチェックリスト

他の開発者や別のPCで使用する場合、以下を確認：

- [ ] `agent.md` が存在し、日本語で書かれている
- [ ] `build.config.json` が存在する
- [ ] `build.config.json` にコメントやガイドが記載されている
- [ ] `SETUP.md` が存在する（このファイル）
- [ ] パスは例示的な値（修正が必要な状態）で配布

---

## サポート

設定に問題がある場合：

1. `build.config.json` を確認
2. パスの存在を確認
3. SETUP.md のトラブルシューティングを参照
4. IDE でプロジェクトを一度ビルドして、Debug フォルダを生成

---

**最終更新**: 2026年2月26日 | **バージョン**: 1.0
