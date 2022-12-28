import math
import json

# スマートポール16進数バイナリデータ
class SmartPoleBinaryData():
    def __init__(self,byte):
        # byte => 16進数文字列
        self.hex = byte.hex()  
        self.data = dict()

    # jsonで書出す
    def writeOutJson(self,file):
        d = self.data
        with open(file, 'w', encoding="utf-8") as f:
            json.dump(d, f, indent=2, ensure_ascii=False)

    # 翻訳する
    def translate(self):

        # 16進数文字列から10進数
        def hexToDecimal(data):
            # print(data,"=>",int(data, 16))
            return int(data, 16)

        s = self.hex

        d = {
            "共通領域":{
                "共通領域アプリヘッダ領域":{
                    "共通領域管理情報":{
                        "共通サービス規格ID・メッセージID・バージョン":hexToDecimal(s[0:4]),
                        "送信識別ID1":hexToDecimal(s[4:6]),
                        "送信識別ID2":hexToDecimal(s[6:8]),
                        "送信識別ID3":hexToDecimal(s[8:10]),
                        "インクリメントカウンタ":hexToDecimal(s[10:12]),
                        "共通アプリデータ長":hexToDecimal(s[12:14]),
                        "オプションフラグ":hexToDecimal(s[14:16])
                    }
                },
                "共通アプリデータ領域":{
                    "時刻情報":{
                        "サマータイム・うるう秒・月":hexToDecimal(s[16:18]),
                        "日":hexToDecimal(s[18:20]),
                        "時":hexToDecimal(s[20:22]),
                        "分":hexToDecimal(s[22:24]),
                        "秒":math.floor(hexToDecimal(s[24:26]) / 4),
                        "ms":hexToDecimal(s[26:28])
                    },
                    "ステータス情報":{
                        "ステータス異常情報":hexToDecimal(s[28:32])
                    },
                    "交差点情報":{
                        "交差点情報":s[32:50]
                    },
                    "拡張情報":{
                        "拡張情報":s[50:64]
                    }
                }
            },
            "認識結果領域":{
                "認識結果アプリヘッダ領域":{
                    "認識結果領域管理情報":{
                        "認識結果領域長":hexToDecimal(s[64:68]),
                        "個別アプリデータ数":hexToDecimal(s[68:72])
                    }
                },
                "認識結果アプリデータ領域":{

                }
            }
        }

        n = 0
        i = 1

        while len(s) >= 120 + n:

            # print(len(s),">=",120 + n)

            buf = {
                "オブジェクトID":hexToDecimal(s[72+n:74+n]),
                "認識クラス":hexToDecimal(s[74+n:76+n]),
                "対象の緯度":round(hexToDecimal(s[76+n:84+n]) * 0.0000001, 7),
                "対象の経度":round(hexToDecimal(s[84+n:92+n]) * 0.0000001, 7),
                "認識信頼性 上限":hexToDecimal(s[92+n:93+n]),
                "認識信頼性 下限":hexToDecimal(s[93+n:94+n]),
                "対象の進行方向[度]":hexToDecimal(s[94+n:98+n]) * 0.0125,
                "対象の移動速度[m/s]":hexToDecimal(s[98+n:102+n]) * 0.01,
                "遅延時間":hexToDecimal(s[102+n:106+n]),
                "デバッグ情報":s[106+n:114+n],
                "その他追加オプション情報":s[114+n:120+n]
            }

            d["認識結果領域"]["認識結果アプリデータ領域"][f"個別アプリデータ{i}"] = buf

            n = n + 48
            i = i + 1

        self.data = d

if __name__ == '__main__':

    # from wireshark #1684 84byte
    b = b"\x00\x40\x00\x00\x01\x0f\x18\x01\x08\x09\x0b\x1f\x40\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x30\x00\x02\xd3\x00\x14\xdf\x8f\xb8\x51\xbc\x4c\x77\x0f\x1d\xd8\x00\x00\x00\x00\xf2\x60\x4d\xd3\x00\x00\x00\x7a\x00\x14\xdf\x8f\xa2\x51\xbc\x4b\xb7\x0f\x5c\x2b\x00\x1c\x00\x00\x5c\x88\xc4\x7a\x00\x00\x00"
    
    obj = SmartPoleBinaryData(b)
    obj.translate()

    print(obj.hex)
    print(obj.data)
    obj.writeOutJson("data/data.json")