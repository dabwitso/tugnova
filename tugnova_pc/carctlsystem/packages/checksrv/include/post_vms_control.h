#ifndef post_vms_control_h
#define post_vms_control_h

#include <iostream>
#include <cstring>
#include <string>
#include <algorithm>
#include <ctime>
#include <stdio.h>
#include <curl/curl.h>
#include <vector>

#include <rapidjson/document.h>
#include <util.h>

static const char *str_hdr_ct = "Content-Type: application/json;";
static const char *str_hdr_ac = "Accept: application/json;";
static const char *str_hdr_cs = "charsets: utf-8;";
static const char *str_hdr_ua = "User-Agent: curl/7.49.0;";

static std::string param_url_string;
static int param_timeout_val = 60;

double post_start_time = 0.0;

// 送信用のパラメータ構造体
static struct _post_param_struct
{
  std::string v_id;             // 車両ID
  std::string ipaddr;   // IPアドレス
  std::string coms_st;          // 車両状態
  std::string route_id; // ルートID
  time_t route_last_mod;
  std::vector<int> waypoint_ids;
  std::string error_id;   // 車両で発生中のエラーコード
  std::string battery_scale; //バッテリ目盛り
  std::string battery_color; //バッテリ背景色
} post_param_struct;

//
// 停止ポイントにおける上位との通信インタフェースを実装する
// curl使ってHTTP通信する（同期型）
//

// 指定の文字列を小文字変換する
std::string to_lower_case(std::string &src_)
{
  std::string dst_;
  dst_.resize(src_.size());
  std::transform(src_.begin(), src_.end(), dst_.begin(), ::tolower);
  return dst_;
}

// POST レスポンスデータの構築処理（定型的な処理）
size_t callback_write(char *ptr, size_t size, size_t nmemb, std::string *stream)
{
  int dataLength = size * nmemb;
  stream->append(ptr, dataLength);
  return dataLength;
}

// 上位サーバへのPOST実行処理
//  i : パラメータ
//  o : レスポンス
int send_post_vms_control(struct _post_param_struct &params_, rapidjson::Document &res_doc_, int post_timeout, int post_force_timeout)
{
  
  // データ部（JSON形式）作成
  std::string param_string = "{";
  param_string += ("\"vehicleId\":\"" + params_.v_id + "\",");
  param_string += ("\"address\":\"" + params_.ipaddr + "\",");
  param_string += ("\"status\":\"" + params_.coms_st + "\",");
  param_string += ("\"route\":{\"routeId\":\"" + params_.route_id + "\",\"lastModified\":" + std::to_string(params_.route_last_mod) + "},");
  param_string += "\"waypointIds\":[";
  std::for_each(params_.waypoint_ids.begin(), params_.waypoint_ids.end(), [&param_string](int waypoint_id) {
    param_string += (std::to_string(waypoint_id) + ",");
  });
  if (param_string.length() >= 1) {
    param_string.erase(param_string.end()-1, param_string.end()); // 終端のカンマを除去
  }
  param_string += "],";
  param_string += ("\"error\":\"" + params_.error_id + "\",");
  param_string += ("\"batteryScale\":" + params_.battery_scale + ",");
  param_string += ("\"batteryColor\":" + params_.battery_color + ",");

  param_string += ("\"sendTime\":" + std::to_string(time(0)));
  param_string += "}";

  ROS_WARN("send_post_vms_control: %s", param_string.c_str());

  int ret = -1;
  CURL *curl;
  curl = curl_easy_init();
  if (curl)
  {
    std::string chunk;
    CURLcode curl_ret;

    // ヘッダー定義
    struct curl_slist *headers = NULL;
    headers = curl_slist_append(headers, str_hdr_ct);
    headers = curl_slist_append(headers, str_hdr_ac);
    headers = curl_slist_append(headers, str_hdr_cs);
    headers = curl_slist_append(headers, str_hdr_ua);

    // パラメータ設定
    curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, "POST");
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_URL, param_url_string.c_str());
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, param_string.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, callback_write);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &chunk);
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, post_timeout);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, post_force_timeout);

    post_start_time = calc_time();

    // リクエスト送信
    curl_ret = curl_easy_perform(curl);
    if (curl_ret == CURLE_OK)
    {
      long response_code;
      curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &response_code);
      // fprintf(stdout, "DEBUG: send_post_vms_control: post success! response code=[%ld]\n", response_code);
      if (response_code == 200)
      {
        ROS_WARN("send_post_vms_control: chunk=[%s]", chunk.c_str());
        res_doc_.Parse(chunk.c_str());
        bool parse_error = res_doc_.HasParseError();
        if (parse_error)
        {
          // fprintf(stderr, "DEBUG: send_post_vms_control: Failed to JSON parse.\n");
        }
        else
        {
          ret = 0; // 正常終了！
        }
      }
    }
    else if (curl_ret == CURLE_OPERATION_TIMEDOUT)
    {
      //通信開始からの経過時間で通信タイムアウトか返信タイムアウトか決める
  		double diff_time = calc_time() - post_start_time;
  		ROS_WARN("send_post_vms_control: diff_time=[%if]", diff_time);
  		if ((post_force_timeout * 1000.0) <= diff_time)
  		{
  			ROS_WARN("response time out");
        //返信タイムアウトの場合は他の通信失敗と区別するために-2を返す
        ret = -2;
  		}
    }
    else
    {
      // fprintf(stderr, "DEBUG: send_post_vms_control: Failed to curl_easy_perform(). [%s]\n", curl_easy_strerror(curl_ret));
    }

    curl_easy_cleanup(curl);
  }
  else
  {
    // fprintf(stderr, "DEBUG: send_post_vms_control: Failed to curl_easy_init().\n");
  }
  return ret;
}

// 通信処理の初期化処理
void init_post_vms_control(const char *url_str_, int tout_val_)
{
  curl_global_init(CURL_GLOBAL_SSL);
  param_url_string = std::string(url_str_);
  param_timeout_val = tout_val_;
}

// 通信理の終了処理
void cleanup_post_vms_control()
{
  curl_global_cleanup();
}

/**
 * レスポンスのresults部の解析結果を返却
 */
void getResultOfWaypoint(
  rapidjson::Value& results, int waypoint_id, bool& out_permission, std::string& out_waitingFor,
  std::string& out_trafficLightState, bool& out_stop, bool& out_forceStop) {
    bool hit = false;

    for (rapidjson::Value::ConstValueIterator array_itr = results.Begin(); array_itr != results.End(); array_itr++) {
        const rapidjson::Value& data = *array_itr;

        for (rapidjson::Value::ConstMemberIterator attribute_itr = data.MemberBegin(); attribute_itr != data.MemberEnd(); attribute_itr++) {
            const std::string name(attribute_itr->name.GetString());
            const rapidjson::Value& value = attribute_itr->value;

            if (name == "waypointId") {
                if (waypoint_id == value.GetInt()) {
                    hit = true;
                } else {
                    break;
                }
            } else if (name == "permission") {
                out_permission = value.GetBool();
            } else if (name == "waitingFor") {
                out_waitingFor = value.GetString();
            } else if (name == "trafficLightState") {
                out_trafficLightState = value.GetString();
            } else if (name == "stop") {
              out_stop = value.GetBool();
            } else if (name == "forceStop") {
              out_forceStop = value.GetBool();
            } else {
                ROS_ERROR("Unknown Attribute: %s", name.c_str());
            }
        }

        if (hit) { return; } // 基本的に必ず結果内に指定したWaypointの情報が含まれている。含まれていない場合は異常である。
    }

    ROS_ERROR("Cannot Hit Result of WaypointID: %d", waypoint_id);
}

#endif // post_vms_control
