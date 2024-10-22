#include <bits/stdc++.h>
#include <iostream>
#include <getopt.h>
#include <fstream>
#include <sstream>
#include <queue> 
#include <chrono>
#include <cstdlib>
#include <algorithm>
#include <iomanip>
#include "graph.h" 
using namespace std;


// Função para escrever distâncias e caminhos no arquivo de texto
void escreve_distancias_e_caminhos_no_arquivo(const vector<pair<float, vector<unsigned int>>>& dist_e_caminhos, const string& nome_arquivo) {
    // Abre o arquivo para escrita
    ofstream arquivo(nome_arquivo);
    
    // Verifica se o arquivo foi aberto corretamente
    if (!arquivo.is_open()) {
        cerr << "Erro ao abrir o arquivo: " << nome_arquivo << endl;
        return;
    }

    // Escreve as distâncias e caminhos no arquivo
    for (size_t i = 0; i < dist_e_caminhos.size(); ++i) {
        arquivo << "Distância de 10 a " << ((i+1)*10+10) << ": " << dist_e_caminhos[i].first << "\n";
        arquivo << "Caminho de 10 a " << ((i+1)*10+10) << ":\n";
        for (const auto& vertice : dist_e_caminhos[i].second) {
            arquivo << vertice << " ";
        }
        arquivo << "\n\n";
    }

    // Fecha o arquivo
    arquivo.close();
    cout << "Distâncias e caminhos gravados com sucesso em " << nome_arquivo << endl;
}


double media_Dijkstra_semHeap(int vezes, const string &arquivo, bool matriz){
        Grafo grafo(arquivo,matriz,true);
        int i = 0;
        double acumulado_DsH = 0;
        while (i<vezes){
            unsigned int s = std::rand() % grafo.getVertices();
            auto inicio = std::chrono::high_resolution_clock::now();
            grafo.algoritmo_Dijkstra(s,"a.txt",0,0);
            auto fim = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> duracao = fim - inicio;
            acumulado_DsH = acumulado_DsH + duracao.count();
            i++;
        }
        return acumulado_DsH;
    }

double media_Dijkstra_comHeap(int vezes, const string &arquivo, bool matriz){
        Grafo grafo(arquivo,matriz,true);
        int i = 0;
        double acumulado_DcH = 0;
        while (i<vezes){
            unsigned int s = std::rand() % grafo.getVertices();
            auto inicio = std::chrono::high_resolution_clock::now();
            grafo.algoritmo_Dijkstra(s,"a.txt",0,1);
            auto fim = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> duracao = fim - inicio;
            acumulado_DcH = acumulado_DcH + duracao.count();
            i++;
        }
        return acumulado_DcH;
    } 

int main(int argc, char *argv[]) {
    //fazer isso para os 5 grafos 
    // (vezes,grafo,matriz,peso)
    
    Grafo grafo_teste("grafo_W_1.txt", false, true);
    
    // Vetor para armazenar distâncias e caminhos
    vector<pair<float, vector<unsigned int>>> dist_e_caminhos;

    // Calcula as distâncias e caminhos de 10 até os outros vértices
    dist_e_caminhos.push_back({grafo_teste.distancia_peso(10, 20, true), grafo_teste.caminho_minimo_peso(10, 20, true)});
    dist_e_caminhos.push_back({grafo_teste.distancia_peso(10, 30, true), grafo_teste.caminho_minimo_peso(10, 30, true)});
    dist_e_caminhos.push_back({grafo_teste.distancia_peso(10, 40, true), grafo_teste.caminho_minimo_peso(10, 40, true)});
    dist_e_caminhos.push_back({grafo_teste.distancia_peso(10, 50, true), grafo_teste.caminho_minimo_peso(10, 50, true)});
    dist_e_caminhos.push_back({grafo_teste.distancia_peso(10, 60, true), grafo_teste.caminho_minimo_peso(10, 60, true)});

    // Escreve as distâncias e caminhos no arquivo de texto
    escreve_distancias_e_caminhos_no_arquivo(dist_e_caminhos, "distancias_e_caminhos_grafo1_comheap.txt");

    // Vetor para armazenar distâncias e caminhos
    vector<pair<float, vector<unsigned int>>> dist_e_caminhos_sh;

    // Calcula as distâncias e caminhos de 10 até os outros vértices
    dist_e_caminhos_sh.push_back({grafo_teste.distancia_peso(10, 20, false), grafo_teste.caminho_minimo_peso(10, 20, false)});
    dist_e_caminhos_sh.push_back({grafo_teste.distancia_peso(10, 30, false), grafo_teste.caminho_minimo_peso(10, 30, false)});
    dist_e_caminhos_sh.push_back({grafo_teste.distancia_peso(10, 40, false), grafo_teste.caminho_minimo_peso(10, 40, false)});
    dist_e_caminhos_sh.push_back({grafo_teste.distancia_peso(10, 50, false), grafo_teste.caminho_minimo_peso(10, 50, false)});
    dist_e_caminhos_sh.push_back({grafo_teste.distancia_peso(10, 60, false), grafo_teste.caminho_minimo_peso(10, 60, false)});

    // Escreve as distâncias e caminhos no arquivo de texto
    escreve_distancias_e_caminhos_no_arquivo(dist_e_caminhos_sh, "distancias_e_caminhos_grafo1_semheap.txt");


    //isso aqui é pra comparar os tempos 
    double sem_heap = media_Dijkstra_semHeap(50,"grafo_W_1.txt",false);
    double com_heap = media_Dijkstra_comHeap(50,"grafo_W_1.txt",false);

    cout << setprecision(6) << sem_heap/50 << endl;
    cout << setprecision(6) << com_heap/50 << endl;

    sem_heap = media_Dijkstra_semHeap(50,"grafo_W_1.txt",true);
    com_heap = media_Dijkstra_comHeap(50,"grafo_W_1.txt",true);

    cout << setprecision(6) << sem_heap/50 << endl;
    cout << setprecision(6) << com_heap/50 << endl;    









    /*int d = 0;
    //distancia entre 2 pontos
    d = grafo.distancia(20,"output.txt",false,30);
    cout << d << endl;
    double acumulado_bfs=0;
    double acumulado_dfs=0;
    //media os tempos de dfs e bfs 
    for (int i =0;i<101;i++){

        unsigned int s = std::rand() % grafo.getVertices();
        auto inicio = std::chrono::high_resolution_clock::now();
        grafo.BFS(s,"a.txt",0);
        auto fim = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duracao = fim - inicio;
        acumulado_bfs= acumulado_bfs + duracao.count();
        std::cout << "Tempo de execução BFS: " << duracao.count() << " segundos" << std::endl;

        s = std::rand() % grafo.getVertices();
        inicio = std::chrono::high_resolution_clock::now();
        grafo.DFS(s,"a.txt",0);
        fim = std::chrono::high_resolution_clock::now();
        duracao = fim - inicio;
        acumulado_dfs= acumulado_dfs + duracao.count();
        std::cout << "Tempo de execução DFS: " << duracao.count() << " segundos" << std::endl;
    }
    cout<< std::setprecision(6) << acumulado_bfs/100 << endl;
    cout << std::setprecision(10) << acumulado_dfs/100 << endl;

    // Nome dos arquivos para salvar as árvores de BFS e DFS
    string treeFile_bfs = "bfs_tree.txt";  
    string treeFile_dfs = "dfs_tree.txt";
        
    grafo.BFS(1,"1" + treeFile_bfs,true);
    grafo.DFS(1,"1" + treeFile_dfs,true);
    grafo.BFS(2,"2" + treeFile_bfs,true);
    grafo.DFS(2,"2" + treeFile_dfs,true);
    grafo.BFS(3,"3" + treeFile_bfs,true);
    grafo.DFS(3,"3" + treeFile_dfs,true);

    //achar e imprimir componentes conexas (ordem decrescente) 
    map<unsigned int, vector<vector<unsigned int>>, greater<unsigned int>> Comp_do_Grafo = grafo.ComponentesConexas();
    grafo.imprimirComponentesConexas(Comp_do_Grafo);
    cout << "Numero de Componentes Conexas: " << grafo.n_CompConexa << endl;*/
        

    return 0;
}

// g++ -O3 -o a.exe main.cpp graph.cpp
// ./a.exe --file grafo_0.txt --useMatrix nao
