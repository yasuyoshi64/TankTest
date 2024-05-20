#include "save_data.hpp"
#include "esp_log.h"

#define TAG "SaveData"

#define SAVE_FILE "/save"

SaveData::SaveData() {
    m_rootPath[0] = '\0';
    m_saveDataMap.clear();
}

void SaveData::init(const char* root) {
    strcpy(m_rootPath, root);
    m_saveDataMap.clear();
}

void SaveData::read() {
    char* szPath = new char[strlen(m_rootPath) + strlen(SAVE_FILE) + 1];
    sprintf(szPath, "%s%s", m_rootPath, SAVE_FILE);
    m_saveDataMap.clear();
    FILE* fd = fopen(szPath, "r");
    if (fd != NULL) {
        char szLine[256];
        int len = 0;
        std::string key, value;
        while(1) {
            int c = fgetc(fd);
            if (feof(fd) || ferror(fd)) {
                break;
            }
            if (c == '\r') {
                continue;
            } else if(c == '=' || c == '\n') {
                szLine[len] = '\0';
                value = szLine;
                if (c == '=') {
                    key = szLine;
                } else {
                    value = szLine;
                    ESP_LOGI(TAG, "key=%s, value=%s", key.c_str(), value.c_str());
                    m_saveDataMap.insert(std::make_pair(key, value));
                }
                len = 0;
            } else {
                szLine[len++] = (char)c;
            }
        }
        fclose(fd);
    }
    delete[] szPath;
}

void SaveData::save() {
    char* szPath = new char[strlen(m_rootPath) + strlen(SAVE_FILE) + 1];
    sprintf(szPath, "%s%s", m_rootPath, SAVE_FILE);
    FILE* fd = fopen(szPath, "w");
    if (fd != NULL) {
        auto iter = m_saveDataMap.begin();
        while(iter != m_saveDataMap.end()) {
            fprintf(fd, "%s=%s\n", iter->first.c_str(), iter->second.c_str());
            iter++;
        }
        fclose(fd);
    }
    delete[] szPath;
}

const char* SaveData::get(const char* key) {
    if (m_saveDataMap.find(key) == m_saveDataMap.end())
        return NULL;
    return m_saveDataMap[key].c_str();
}

void SaveData::set(const char* key, const char* value) {
    m_saveDataMap[key] = value;
}
