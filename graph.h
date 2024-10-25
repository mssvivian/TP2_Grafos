#ifndef GRAPH_H
#define GRAPH_H

#include <bits/stdc++.h>
#include <iostream>
#include <getopt.h>
#include <fstream>
#include <sstream>
#include <queue> 
#include <chrono>
#include <algorithm>
using namespace std;

class Grafo {
private:
    vector<vector<float>> matrizAdj_peso;
    vector<vector<tuple<unsigned int,float>>> listAdj_peso;
    vector<vector<bool>> matrizAdj;
    vector<vector<unsigned int>> listAdj;
    unsigned int vertices;
    unsigned int arestas;
    bool grafo_peso;
    bool usaMatriz;
    vector<bool> visitado;
    vector<int> pai;
    vector<int> nivel;
    vector<float> dist;
    unsigned int n_CompConexa; 

public:
    // Construtor
    Grafo(const string &grafo, bool isMatriz, bool peso);

    // Get vértices
    unsigned int getVertices();

    // Criar txt
    void write_general_info(const string &arquivo);
    
    // Graus de um vértice
    tuple<unsigned int, unsigned int, unsigned int, float> graus();

    // Imprime a Matriz de Adjacência
    void printMatrizAdj() const;

    // Imprime a Lista de Adjacência
    void printListAdj() const;

    // Escrever em arquivo (para retornar a árvore gerada por busca)
    void Write_file_busca(vector<int> pai, vector<int> nivel, const string& outputFile, unsigned int s);

    // Escrever em arquivo (para retornar a árvore gerada por busca) -- grafo com peso
    void Write_file_busca_peso(vector<int> pai, vector<float> dist, const string& outputFile, unsigned int s);

    // Obtém os vizinhos de um vértice na matriz de adjacência
    vector<unsigned int> getVizinhosMatriz(unsigned int v) const;

    // Obtém os vizinhos de um vértice na matriz de adjacência com pesos
    vector<tuple<unsigned int,float>> getVizinhosMatriz_peso(unsigned int v) const;

    // Algoritmo Diâmetro aproximado
    int aprox(unsigned int start, const string& outputFile, bool write_tree);

    // Algoritmo diâmetro não aproximado
    int diameter();

    // Algoritmo Distância 
    int distancia(unsigned int start, const string& outputFile, bool write_tree, unsigned int end);

    // Algoritmo BFS
    void BFS(unsigned int s, const std::string& outputFile, bool write_tree, unsigned int e = 0);

    // Algoritmo DFS
    void DFS(unsigned int s, const std::string& outputFile, bool write_tree);

    // Componentes Conexas
    map<unsigned int, vector<vector<unsigned int>>, greater<unsigned int>> ComponentesConexas();

    // Imprimir Componentes Conexas
    void imprimirComponentesConexas(const map<unsigned int, vector<vector<unsigned int>>, greater<unsigned int>>& componentesConexas);

    // Algoritmo de Dijkstra que calcula a distancia de 1 vertice para todos os outros, retorna o vetor de distancia e a arvore geradora 
    vector<float> algoritmo_Dijkstra(unsigned int s, const string& outputFile, bool write_tree, bool Heap);

    // funçao distancia -- entra com booleano de heap, vertice inicial e final, retorna a distancia entre esses 2 pontos
    float distancia_peso(unsigned int start, unsigned int end, bool Heap);

    // funçao distancia -- entra com booleano de heap, vertice inicial e final, retorna a distancia entre esses 2 pontos
    vector<unsigned int> caminho_minimo_peso(unsigned int start, unsigned int end, bool Heap);

    // função transforma o caminho mínimo dos índices para o caminho mínimo com o nome de cada pesquisador
    vector <string> Grafo::caminho_minimo_nomes(std::vector<std::string> cod, vector<int> caminho);

};
#endif // GRAPH_H
