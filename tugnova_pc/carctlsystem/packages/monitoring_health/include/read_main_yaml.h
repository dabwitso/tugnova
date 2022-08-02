#ifndef read_main_yaml_h
#define read_main_yaml_h

#include <string>
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>

//
// 設定ファイル（main.yaml）の読み込み処理を共通化する
// getterを用意して随時参照してもらう
//

// 設定内容

// process:
std::string main_yaml_process_timer = 60;
std::string main_yaml_process_device_name = "";

// 指定文字列から改行コードを削除する
//  i/o : 変換対象文字列
void deleteNl(std::string &targetStr_)
{
  const char CR = '\r';
  const char LF = '\n';
  std::string destStr_;
  for (std::string::const_iterator it = targetStr_.begin(); it != targetStr_.end(); ++it)
  {
    if (*it != CR && *it != LF)
    {
      destStr_ += *it;
    }
  }
  targetStr_ = destStr_;
}

// 指定ファイルの1行目を読み込んで文字列として返却する
//  i : 読み込み対象のテキストファイル名（フルパス）
//  o : 読み込んだ1行目の文字列
std::string read_one_line(std::string ifname_)
{
  std::ifstream ifs(ifname_.c_str());
  std::string str = "";
  if (ifs.fail())
  {
    fprintf(stderr, "read_one_line: Failed to open file. [%s]\n", ifname_.c_str());
  }
  else
  {
    getline(ifs, str);
    deleteNl(str);
  }
  return str;
}

// main.yamlを読み込んでグローバル変数へ格納する
//  i : 読み込み対象のファイル名（フルパス）
//  o : 0:正常
int read_main_yaml(std::string fname_)
{

  // Load file
  YAML::Node lconf = YAML::LoadFile(fname_.c_str());

  main_yaml_process_timer = lconf["timer"].as<int>();
  main_yaml_process_device_name = lconf["device_name"].as<std::string>();

  fprintf(stdout, "DEBUG:   post: timer: [%s]\n", main_yaml_process_timer.c_str());
  fprintf(stdout, "DEBUG:   post: device_name: [%s]\n", main_yaml_process_device_name);

  //fprintf(stdout, "DEBUG:   process: SlowDown: [%g]\n", main_yaml_process_SlowDown);
  return 0;
}

#endif // read_main_yaml
