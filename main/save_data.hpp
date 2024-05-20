/**
 * データ保存
 * SDカードの/saveファイルに保存/読み込みを行う。
*/
#pragma once

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <map>

class SaveData {
    public:
        SaveData();

    public:
        void init(const char* root);
        void read();
        void save();
        const char* get(const char* key);
        void set(const char* key, const char* value);

    private:
        char m_rootPath[256];
        std::map<std::string, std::string> m_saveDataMap{};     // 保存データ
};
