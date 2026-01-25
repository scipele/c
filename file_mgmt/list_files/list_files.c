#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#ifdef _WIN32
#include <windows.h>
#include <shlobj.h>
#else
#include <dirent.h>
#include <sys/stat.h>
#endif
#include <openssl/sha.h>

typedef struct FileInfo {
    char* folder;
    char* filename;
    char* time_str;
    char* hash;
    struct FileInfo* next;
} FileInfo;

typedef struct HashCount {
    char* hash;
    int count;
    struct HashCount* next;
} HashCount;

char* compute_sha1(const char* filepath) {
    FILE* file = fopen(filepath, "rb");
    if (!file) return NULL;
    SHA_CTX sha_ctx;
    SHA1_Init(&sha_ctx);
    unsigned char buffer[1024];
    size_t bytes_read;
    while ((bytes_read = fread(buffer, 1, sizeof(buffer), file)) > 0) {
        SHA1_Update(&sha_ctx, buffer, bytes_read);
    }
    unsigned char hash[SHA_DIGEST_LENGTH];
    SHA1_Final(hash, &sha_ctx);
    fclose(file);
    char* hex_hash = malloc(SHA_DIGEST_LENGTH * 2 + 1);
    for (int i = 0; i < SHA_DIGEST_LENGTH; i++) {
        sprintf(hex_hash + i*2, "%02x", hash[i]);
    }
    hex_hash[SHA_DIGEST_LENGTH * 2] = '\0';
    return hex_hash;
}

void list_files_posix(const char *path, FileInfo** head);

#ifdef _WIN32
void list_files_win(const char *path, FileInfo** head);
#endif

void list_files(const char *path, FileInfo** head) {
#ifdef _WIN32
    list_files_win(path, head);
#else
    list_files_posix(path, head);
#endif
}

#ifndef _WIN32
void list_files_posix(const char *path, FileInfo** head) {
    DIR *dir = opendir(path);
    if (!dir) {
        perror("opendir");
        return;
    }
    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) continue;
        char fullpath[1024];
        snprintf(fullpath, sizeof(fullpath), "%s/%s", path, entry->d_name);
        struct stat st;
        if (stat(fullpath, &st) == 0) {
            if (S_ISDIR(st.st_mode)) {
                list_files_posix(fullpath, head);
            } else {
                char *filename = strrchr(fullpath, '/');
                if (filename) {
                    filename++; // point to filename
                    char parent[1024];
                    size_t len = filename - fullpath - 1; // length before /
                    if (len == 0) {
                        strcpy(parent, "/");
                    } else {
                        strncpy(parent, fullpath, len);
                        parent[len] = '\0';
                    }
                    struct tm *tm = localtime(&st.st_mtime);
                    char time_str[20];
                    strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", tm);
                    char* hash = compute_sha1(fullpath);
                    FileInfo* new_info = malloc(sizeof(FileInfo));
                    new_info->folder = strdup(parent);
                    new_info->filename = strdup(filename);
                    new_info->time_str = strdup(time_str);
                    new_info->hash = hash;
                    new_info->next = NULL;
                    if (*head == NULL) {
                        *head = new_info;
                    } else {
                        FileInfo* temp = *head;
                        while (temp->next) temp = temp->next;
                        temp->next = new_info;
                    }
                } else {
                    // no /, just filename
                    struct tm *tm = localtime(&st.st_mtime);
                    char time_str[20];
                    strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", tm);
                    char* hash = compute_sha1(fullpath);
                    FileInfo* new_info = malloc(sizeof(FileInfo));
                    new_info->folder = strdup("");
                    new_info->filename = strdup(fullpath);
                    new_info->time_str = strdup(time_str);
                    new_info->hash = hash;
                    new_info->next = NULL;
                    if (*head == NULL) {
                        *head = new_info;
                    } else {
                        FileInfo* temp = *head;
                        while (temp->next) temp = temp->next;
                        temp->next = new_info;
                    }
                }
            }
        }
    }
    closedir(dir);
}
#endif

#ifdef _WIN32
void list_files_win(const char *path, FileInfo** head) {
    char search_path[1024];
    snprintf(search_path, sizeof(search_path), "%s\\*", path);
    WIN32_FIND_DATA find_data;
    HANDLE hFind = FindFirstFile(search_path, &find_data);
    if (hFind == INVALID_HANDLE_VALUE) {
        return;
    }
    do {
        if (strcmp(find_data.cFileName, ".") == 0 || strcmp(find_data.cFileName, "..") == 0) continue;
        char fullpath[1024];
        snprintf(fullpath, sizeof(fullpath), "%s\\%s", path, find_data.cFileName);
        if (find_data.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) {
            list_files_win(fullpath, head);
        } else {
            char *filename = strrchr(fullpath, '\\');
            if (filename) {
                filename++;
                char parent[1024];
                size_t len = filename - fullpath - 1;
                if (len == 0) {
                    strcpy(parent, path); // root
                } else {
                    strncpy(parent, fullpath, len);
                    parent[len] = '\0';
                }
                FILETIME ft = find_data.ftLastWriteTime;
                SYSTEMTIME st;
                FileTimeToSystemTime(&ft, &st);
                char time_str[20];
                sprintf(time_str, "%04d-%02d-%02d %02d:%02d:%02d", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
                char* hash = compute_sha1(fullpath);
                FileInfo* new_info = malloc(sizeof(FileInfo));
                new_info->folder = _strdup(parent);
                new_info->filename = _strdup(filename);
                new_info->time_str = _strdup(time_str);
                new_info->hash = hash;
                new_info->next = NULL;
                if (*head == NULL) {
                    *head = new_info;
                } else {
                    FileInfo* temp = *head;
                    while (temp->next) temp = temp->next;
                    temp->next = new_info;
                }
            } else {
                // no \, just filename
                FILETIME ft = find_data.ftLastWriteTime;
                SYSTEMTIME st;
                FileTimeToSystemTime(&ft, &st);
                char time_str[20];
                sprintf(time_str, "%04d-%02d-%02d %02d:%02d:%02d", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
                char* hash = compute_sha1(fullpath);
                FileInfo* new_info = malloc(sizeof(FileInfo));
                new_info->folder = _strdup("");
                new_info->filename = _strdup(fullpath);
                new_info->time_str = _strdup(time_str);
                new_info->hash = hash;
                new_info->next = NULL;
                if (*head == NULL) {
                    *head = new_info;
                } else {
                    FileInfo* temp = *head;
                    while (temp->next) temp = temp->next;
                    temp->next = new_info;
                }
            }
        }
    } while (FindNextFile(hFind, &find_data));
    FindClose(hFind);
}
#endif

int main(int argc, char *argv[]) {
    char path[1024] = "";
#ifdef _WIN32
    // Use folder picker on Windows
    CoInitialize(NULL);
    BROWSEINFO bi = {0};
    bi.lpszTitle = "Select a folder to list files and compute hashes:";
    bi.ulFlags = BIF_RETURNONLYFSDIRS | BIF_NEWDIALOGSTYLE;
    LPITEMIDLIST pidl = SHBrowseForFolder(&bi);
    if (pidl) {
        SHGetPathFromIDList(pidl, path);
        CoTaskMemFree(pidl);
        CoUninitialize();
    } else {
        printf("No folder selected.\n");
        CoUninitialize();
        return 1;
    }
#else
    if (argc > 1) {
        strcpy(path, argv[1]);
    } else {
        printf("Enter path: ");
        fgets(path, sizeof(path), stdin);
        path[strcspn(path, "\n")] = 0; // remove newline
    }
#endif
    FileInfo* head = NULL;
    list_files(path, &head);
    
    // Count hashes
    HashCount* hash_head = NULL;
    FileInfo* current = head;
    while (current) {
        if (current->hash) {
            HashCount* hc = hash_head;
            int found = 0;
            while (hc) {
                if (strcmp(hc->hash, current->hash) == 0) {
                    hc->count++;
                    found = 1;
                    break;
                }
                hc = hc->next;
            }
            if (!found) {
                HashCount* new_hc = malloc(sizeof(HashCount));
                new_hc->hash = strdup(current->hash);
                new_hc->count = 1;
                new_hc->next = hash_head;
                hash_head = new_hc;
            }
        }
        current = current->next;
    }
    
    FILE *fp = fopen("list.txt", "w");
    if (!fp) {
        perror("fopen");
        return 1;
    }
    // print the header
    fprintf(fp, "folder|file_name|last_modified|sha1_hash|duplicate_count\n");
    
    current = head;
    while (current) {
        int count = 1;
        if (current->hash) {
            HashCount* hc = hash_head;
            while (hc) {
                if (strcmp(hc->hash, current->hash) == 0) {
                    count = hc->count;
                    break;
                }
                hc = hc->next;
            }
        }
        fprintf(fp, "%s|%s|%s|%s|%d\n", current->folder, current->filename, current->time_str, current->hash ? current->hash : "", count);
        current = current->next;
    }
    fclose(fp);
    printf("List written to list.txt\n");
    return 0;
}