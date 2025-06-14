#include "assimp.h"
#include "glTF.h"

class Loader {
    public:

    Loader(int screenW = 800, int screenH = 600, const string& filepath = "") {
        if (filepath.empty())
            throw std::runtime_error("No file");

        auto pos = filepath.rfind('.');
        if (pos == std::string::npos)
            throw std::runtime_error("Loader: filename has no extension");
        
        std::string ext = filepath.substr(pos + 1);
        std::transform(ext.begin(), ext.end(), ext.begin(),
                       [](unsigned char c){ return std::tolower(c); });
        if (std::find(SupportedExtensions.begin(), SupportedExtensions.end(), ext) != SupportedExtensions.end()) {
            parser = AssImpParser(screenW, screenH, filepath);
        } else {
            throw std::runtime_error("Loader: unsupported file extension: " + ext);
        }

    }

    inline static const std::vector<std::string> SupportedExtensions = {
        "gltf",
        "glb",
        "dae",
        "obj"
    };
   
    Parser parser;
};