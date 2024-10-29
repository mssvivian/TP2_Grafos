#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include <vector>
#include <tuple> // Para retornar múltiplos valores

// Função para ler o arquivo e preencher o mapa e o vetor
std::tuple<std::map<std::string, int>, std::vector<std::string>> lerArquivoParaMapEVetor(const std::string& nomeArquivo) {
    std::map<std::string, int> mapa;
    std::vector<std::string> vetor;
    std::ifstream arquivo(nomeArquivo);
    std::string linha;

    while (std::getline(arquivo, linha)) {
        std::stringstream ss(linha);
        std::string indice_str, nome;
        
        while (std::getline(ss, indice_str, ',') && std::getline(ss, nome)) {
            int indice = std::stoi(indice_str);
            mapa[nome] = indice;
            if (indice >= vetor.size()) {
                vetor.resize(indice + 1);
            }
            vetor[indice] = nome;
        }
    }

    return {mapa, vetor};
}

int main() {
    std::string nomeArquivo = "rede_colaboracao_vertices.txt";
    auto [mapa, vetor] = lerArquivoParaMapEVetor(nomeArquivo);
    int indiceDesejado = 2;
    std::cout << mapa["Alan M. Turing"];
    std::cout << vetor[11365];

    return 0;
}

//g++ -O3 -o a.exe map.cpp
//./a.exe