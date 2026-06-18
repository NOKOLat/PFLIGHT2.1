# C++ Format Guide

この文書は `Core/Lib/ICM42688P/ICM42688P.h` と `ICM42688P.cpp` から読み取れる、コードの見た目に関する規約です。

実装方針や設計パターンではなく、改行、インデント、空白、括弧位置、コメント配置をそろえるための情報だけをまとめます。

## 基本

- インデントはスペース 4 個を基本にする。
- タブは使わない。ただし既存コードに混ざっている箇所がある場合は、新規追加分ではスペースにそろえる。
- 1 文は基本的に 1 行で書く。
- 関数、制御構文、まとまった処理ブロックの間には空行を入れる。
- 行末の余分な空白は入れない。

## ファイル先頭

ファイル先頭には C 形式のブロックコメントを置く。

```cpp
/*
 * ICM42688P.cpp
 *
 *  Created on: Mar 6, 2025
 *      Author: Sezakiaoi
 */
```

コメントブロックの後に 1 行空けて、`#include` を書く。

```cpp
//#include "stdio.h"
#include "ICM42688P.h"
```

## Include

- `#include` はファイル先頭にまとめる。
- C++ 標準ヘッダは `<...>`、自作ヘッダは `"..."` を使う。
- include 群の中では空行を多用しない。

```cpp
#include <cstdint>
#include <bitset>
#include <cmath>
```

## クラス定義

クラス名の直後に `{` を置き、`{` の前には空白を入れない。

```cpp
class ICM42688P{

    public:

        // ...

    private:

        // ...
};
```

アクセス指定子 `public:` / `private:` は 4 スペースでインデントする。  
その中のメンバはさらに 4 スペース深くし、合計 8 スペースでインデントする。

## enum class

`enum class` は型名の後に `: uint8_t` を付け、`{` の前には空白を入れない。

```cpp
enum class GYRO_MODE: uint8_t{

    OFF = 0x00,
    Standby = 0x01,
    LowNoize = 0x03
};
```

書き方の特徴:

- `enum class NAME: uint8_t{` の形にする。
- `{` の直後に空行を入れる。
- 列挙子は 4 スペース深くする。
- 列挙子の代入演算子 `=` の左右には空白を入れる。
- 最後の列挙子にカンマがある enum と無い enum が混在しているため、新規コードではどちらかに統一する。既存に合わせるなら最後のカンマ無しを基本にする。

## 関数宣言

関数宣言は 1 行で書く。引数が長くても、既存コードでは折り返さず横に伸ばしている。

```cpp
uint8_t GetRawData(int16_t accel_buffer[3], int16_t gyro_buffer[3]);
uint8_t AccelConfig(ICM42688P::ACCEL_Mode accel_mode, ICM42688P::ACCEL_SCALE accel_scale, ICM42688P::ACCEL_ODR accel_odr, ICM42688P::ACCEL_DLPF accel_dlpf);
```

型と関数名の間には空白を 1 個入れる。

## 関数定義

関数定義でも `{` は同じ行に置く。`{` の前には空白を入れない。

```cpp
uint8_t ICM42688P::Connection(){

    uint8_t product_id = 0x00;
    uint8_t error = 0;

    // ...
}
```

書き方の特徴:

- 関数シグネチャと `{` は同じ行。
- `{` の直後に空行を入れる。
- 関数本体は 4 スペースでインデントする。
- `}` の前にも、処理のまとまりに応じて空行が入ることがある。

## 制御構文

`if` / `while` / `for` は、キーワードと `(` の間に空白を入れない。

```cpp
while(product_id != 0x47){

    Read((uint8_t)ICM42688P::BANK0::WHO_AM_I, &product_id, 1);
    error ++;

    if(error > 100){

        log("[ICM42688P] Not Found\n");
        return 1;
    }
}
```

書き方の特徴:

- `if(condition){` の形にする。
- `while(condition){` の形にする。
- `for(uint8_t i = 0; i < 3; i++){` の形にする。
- `{` の直後に空行を入れる傾向がある。
- ネストすると 4 スペースずつ深くする。

## 空行

空行は多めに使う。

主に次の位置に空行を入れる。

- 関数と関数の間
- コメントブロックと関数定義の間
- 変数宣言のまとまりの後
- `while` / `if` / `for` の `{` 直後
- `return` の前
- レジスタ設定など、処理の段落が切り替わる場所

例:

```cpp
uint8_t error = 0;
uint8_t now_mode = 0;
while(command != now_mode){

    Write((uint8_t)ICM42688P::BANK0::PWR_MGMT0, &command, 1);
    Read((uint8_t)ICM42688P::BANK0::PWR_MGMT0, &now_mode, 1);

    error ++;
    if(error > 100){

        log("[ICM42688P] AccelConfig: PWR_MGMT0 Setting Failed\n");
        return 1;
    }
}
```

## 空白

### 演算子

多くの二項演算子では左右に空白を入れる。

```cpp
uint8_t product_id = 0x00;
uint8_t error = 0;
command = gyro_dlpf_tmp | (accel_dlpf_tmp << 4);
```

新規コードでは、読みやすさのために以下のようにそろえる。

```cpp
accel_scale_value = (16.0 / pow(2, (uint8_t)accel_scale)) / 32768 * 9.8;
```

### インクリメント

既存コードでは後置インクリメントの前に空白が入っている。

```cpp
error ++;
```

ただし C++ の一般的なフォーマットでは `error++;` が多い。既存コードに寄せるなら `error ++;`、新規に統一するなら `error++;` を選ぶ。

このプロジェクトの既存 ICM42688P に合わせる場合は、`error ++;` の形にする。

### キャスト

C スタイルキャストを使い、キャスト直後に空白を入れない。

```cpp
(uint8_t)accel_mode
(int16_t)(raw_data[1] | (raw_data[0] << 8))
```

## 代入の桁そろえ

連続した代入では、`=` の位置をそろえる箇所がある。

```cpp
accel_buffer[0]  = (int16_t)(raw_data[1] | (raw_data[0] << 8)) - accel_offset[0];
accel_buffer[1]  = (int16_t)(raw_data[3] | (raw_data[2] << 8)) - accel_offset[1];
accel_buffer[2]  = (int16_t)(raw_data[5] | (raw_data[4] << 8)) - accel_offset[2];

gyro_buffer[0]  = (int16_t)(raw_data[7]  | raw_data[6]  << 8) - gyro_offset[0];
gyro_buffer[1]  = (int16_t)(raw_data[9]  | raw_data[8]  << 8) - gyro_offset[1];
gyro_buffer[2]  = (int16_t)(raw_data[11] | raw_data[10] << 8) - gyro_offset[2];
```

配列の軸ごとの処理など、同じ形が並ぶ場所では桁そろえを許可する。

## コメント

関数の前には Doxygen 風のブロックコメントを置く。

```cpp
/* @brief センサとの接続を確認
 *
 * @return uint8_t 0: 成功, 1: 失敗
 */
uint8_t ICM42688P::Connection(){
```

コメントの形:

- `/* @brief ...` で開始する。
- 2 行目以降は ` * ` でそろえる。
- 引数は `@param [in]` / `@param [out]` を使う。
- 戻り値は `@return` を使う。

短い補足コメントは行末にも書かれている。

```cpp
return 1;//接続失敗
```

ただし新規コードでは、可読性のためにコメント前に空白を入れる。

```cpp
return 1; // 接続失敗
```

## private メンバのグループ化

private メンバは短い `//` コメントで分類する。

```cpp
private:

        // function
        uint8_t (*Write)(uint8_t reg_addr, uint8_t* tx_buffer, uint8_t len);
        uint8_t (*Read)(uint8_t reg_addr, uint8_t* rx_buffer, uint8_t len);
        void (*log)(const char* msg);

        // Offset
        int16_t accel_offset[3] = {};
        int16_t gyro_offset[3] = {};
```

グループコメントの前後には空行を入れる。

## ヘッダーガード

ヘッダーガードは `#ifndef` / `#define` / `#endif` を使う。

```cpp
#ifndef SRC_ICM42688P_H_
#define SRC_ICM42688P_H_

// declarations

#endif /* SRC_ICM42688P_H_ */
```

`#endif` にはガード名をコメントで付ける。

## 推奨テンプレート

既存スタイルに合わせた最小テンプレート:

```cpp
/* @brief 関数の説明
 *
 * @param [in]value 入力値
 *
 * @return uint8_t 0: 成功, 1: 失敗
 */
uint8_t ClassName::FunctionName(uint8_t value){

    uint8_t error = 0;
    uint8_t now_value = 0;

    while(value != now_value){

        Write((uint8_t)Register::CONFIG, &value, 1);
        Read((uint8_t)Register::CONFIG, &now_value, 1);

        error ++;
        if(error > 100){

            log("[ClassName] FunctionName Failed\n");
            return 1;
        }
    }

    return 0;
}
```

## まとめ

この C++ コードのフォーマット上の特徴は、次のとおりです。

- 4 スペースインデント
- `{` は同じ行、ただし `{` の前に空白を入れない
- `if(` / `while(` / `for(` のようにキーワード後の空白を入れない
- ブロック開始直後に空行を入れる
- 処理の段落ごとに空行を多めに入れる
- 関数前に Doxygen 風コメントを書く
- ヘッダーでは `public:` / `private:` の下を深めにインデントする
- 配列処理など同型の行では桁そろえを使う
