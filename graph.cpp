#include <bits/stdc++.h>
using namespace std;
#include <iostream>
#include <getopt.h>
#include <fstream>
#include <sstream>
#include <queue> 
#include <chrono>
#include <algorithm>
#include "graph.h"


// Definição do construtor
Grafo::Grafo(const string &grafo, bool isMatriz, bool peso)
    : usaMatriz(isMatriz), n_CompConexa(0), grafo_peso(peso) {

    ifstream file(grafo);
    if (!file.is_open()) {
        cerr << "Erro ao abrir o arquivo!" << endl;
        exit(EXIT_FAILURE);
    }

    // Lê o número de vértices
    file >> vertices;
    
    visitado.resize(vertices + 1, false);
    pai.resize(vertices + 1, -1);
    nivel.resize(vertices + 1, -1);
    arestas = 0;
    if (usaMatriz) {
        if (grafo_peso){
            matrizAdj_peso.resize(vertices + 1, vector<float>(vertices + 1, numeric_limits<float>::infinity()));

            // Lê as arestas e preenche a matriz de adjacência
            unsigned int u, v;
            float w;
            while (file >> u >> v >> w) {
                matrizAdj_peso[u][v] = w;
                matrizAdj_peso[v][u] = w; // Se o grafo for não direcionado
                arestas++;
            }
        } else{
            matrizAdj.resize(vertices + 1, vector<bool>(vertices + 1, false));

            // Lê as arestas e preenche a matriz de adjacência
            unsigned int u, v;
            while (file >> u >> v) {
                matrizAdj[u][v] = true;
                matrizAdj[v][u] = true; // Se o grafo for não direcionado
                arestas++;

            }
        }

    } else {
        if (grafo_peso){
            listAdj_peso.resize(vertices + 1);

            // Lê as arestas e preenche a lista de adjacência
            unsigned int u, v;
            float w;
            while (file >> u >> v >> w) {
                tuple<unsigned int, float> t1 = make_tuple(u, w);
                tuple<unsigned int, float> t2 = make_tuple(v, w);
                listAdj_peso[u].push_back(t2);
                listAdj_peso[v].push_back(t1); // Se o grafo for não direcionado
                arestas++;
            }
        } else{
            listAdj.resize(vertices + 1);

            // Lê as arestas e preenche a lista de adjacência
            unsigned int u, v;
            while (file >> u >> v) {
                listAdj[u].push_back(v);
                listAdj[v].push_back(u); // Se o grafo for não direcionado
                arestas++;
            }
        }
    }

    file.close();
    }

// Get vertices
unsigned int Grafo::getVertices(){
    return vertices;
}


// Graus do grafo (máx,min,mediana e média)
tuple<unsigned int, unsigned int, unsigned int, float> Grafo::graus() {
    unsigned int max = INT_MIN;  // Valor mínimo possível para garantir a atualização
    unsigned int min = INT_MAX;  // Valor máximo possível para garantir a atualização
    unsigned int mediana = 0;
    float media = 0.0;
    vector<int> graus(vertices + 1, -1);
    float total = 0.0;
    if (usaMatriz) {
        for (unsigned int i = 0; i < vertices; i++) {
            graus[i] = accumulate(matrizAdj[i].begin(), matrizAdj[i].end(), 0); // Calcula graus
            total = total + graus[i];
        }
    } else {
        for (unsigned int i = 0; i < vertices; i++) {
            graus[i] = listAdj[i].size(); // Calcula graus na lista de adjacência
            total = total + graus[i];
        }
    }
    sort(graus.begin(),graus.end());
    // Encontrar os graus máximo e mínimo
    max = graus.back();
    min = graus.front();
    int size = graus.size();
    mediana = graus[size/2];
    media = total/vertices;

    tuple<unsigned int, unsigned int, unsigned int, float> resultado = make_tuple(max,min,mediana,media);

    return resultado;
}

// Criar txt
void Grafo::write_general_info(const string &arquivo){
    tuple<unsigned int, unsigned int, unsigned int, float> t = graus();
    ofstream outFile(arquivo);
    if (!outFile.is_open()) {
        cerr << "Erro ao abrir arquivo de saída." << endl;
        return;
    }

    outFile << "Informações gerais do grafo.\n";
    outFile << "Vértices: " << vertices << endl;
    outFile << "Arestas: " << arestas << endl;
    outFile << "Grau máximo: " << get<0>(t) << endl;
    outFile << "Grau mínimo: " << get<1>(t) << endl;
    outFile << "Mediana do grau: " << get<2>(t) << endl;
    outFile << "Média do grau: " << get<3>(t) << endl;
    
    outFile.close();

}

// Imprime a Matriz de Adjacência
void Grafo::printMatrizAdj() const {
    for (size_t i = 1; i < matrizAdj.size(); ++i) {
        for (size_t j = 1; j < matrizAdj[i].size(); ++j) {
            cout << matrizAdj[i][j] << " ";
        }
        cout << endl;
    }
}

// Imprime a Lista de Adjacência
void Grafo::printListAdj() const {
    for (size_t i = 1; i < listAdj.size(); ++i) {
        cout << i << ": ";
        for (size_t j = 0; j < listAdj[i].size(); ++j) {
            cout << listAdj[i][j] << " ";
        }
        cout << endl;
    }
}

//escrever em arquivo (para retornar a arvore gerada por busca)
void Grafo::Write_file_busca(vector<int> pai,vector<int> nivel, const string& outputFile, unsigned int s){
    ofstream outFile(outputFile);
    if (!outFile.is_open()) {
        cerr << "Erro ao abrir arquivo de saída." << endl;
        return;
    }
    outFile << "Árvore gerada por busca\n";

    for (unsigned int v = 1; v <= vertices; v++){
    if (pai[v] != -1 || v == s) {
        outFile << "Vértice: " << v << ", Pai: " << pai[v] << ", Nível: " << nivel[v] << endl;
        }
    }
    outFile.close();
}

// Escrever em arquivo (para retornar a árvore gerada por busca) -- grafo com peso
void Grafo::Write_file_busca_peso(vector<int> pai, vector<float> dist, const string& outputFile, unsigned int s){
    ofstream outFile(outputFile);
    if (!outFile.is_open()) {
        cerr << "Erro ao abrir arquivo de saída." << endl;
        return;
    }
    outFile << "Árvore gerada por busca\n";

    for (unsigned int v = 1; v <= vertices; v++){
    if (pai[v] != -1 || v == s) {
        outFile << "Vértice: " << v << ", Pai: " << pai[v] << ", Distância: " << dist[v] << endl;
        }
    }
    outFile.close();
}

// Obtém os vizinhos de um vértice na matriz de adjacência sem pesos
vector<unsigned int> Grafo::getVizinhosMatriz(unsigned int v) const {
    vector<unsigned int> vizinhos;
    for (unsigned int i = 1; i <= vertices; ++i) {
        if (matrizAdj[v][i] == 1) {
            vizinhos.push_back(i);
        }
    }
    return vizinhos;
}

// Obtém os vizinhos de um vértice na matriz de adjacência com pesos
vector<tuple<unsigned int,float>> Grafo::getVizinhosMatriz_peso(unsigned int v) const {
    vector<tuple<unsigned int,float>> vizinhos;
    for (unsigned int i = 1; i <= vertices; ++i) {
        if (matrizAdj_peso[v][i] != numeric_limits<float>::infinity()) {
            tuple<unsigned int, float> t1 = make_tuple(i, matrizAdj_peso[v][i]);
            vizinhos.push_back(t1);
        }
    }
    return vizinhos;
    
}

// Algoritmo Diâmetro aproximado -- acho que dá pra tirar ou teriamos que redefinir para pesos 
int Grafo::aprox(unsigned int start, const string& outputFile, bool write_tree){
    if (grafo_peso){
        cerr << "Essa função é apenas para grafos sem peso" << endl;
        return 0;
    }
    BFS (start, outputFile, write_tree); // encontrando a primeira extremidade (x)
    int maior = 0;
    int x;
    for (int i = 1; i < nivel.size(); i++){
        if (nivel[i]> maior){
            maior = nivel[i];
            x = i;
        }
    }
    BFS (x, outputFile, write_tree); // encontrando a segunda extremidade começando da primeira extremidade (x)
    for (int i = 1; i < nivel.size(); i++){
        if (nivel[i]> maior){
            maior = nivel[i];
            x = i;
        }
    }
    return maior;
}

// Algoritmo diâmetro não aproximado -- tbm considera sem peso
int Grafo::diameter() {
    if (grafo_peso){
        cerr << "Essa função é apenas para grafos sem peso" << endl;
        return 0;
    }
    int x = 0;
    int maior = 0;

    for (int s = 1; s < vertices; s++) {
        visitado.assign(vertices + 1, false);
        nivel.assign(vertices + 1, -1);

        queue<unsigned int> Q;
        
        visitado[s] = true;
        nivel[s] = 0;
        Q.push(s);

        while (!Q.empty()) {

            unsigned int v = Q.front();
            Q.pop();

            const vector<unsigned int>& vizinhos = usaMatriz ? getVizinhosMatriz(v) : listAdj[v];
            for (unsigned int w : vizinhos) {
                if (!visitado[w]) {
                    visitado[w] = true;
                    nivel[w] = nivel[v] + 1; // Define o nível de w
                    Q.push(w);
                    if (nivel[w]>maior){
                        maior = nivel[w];
                        x = w;
                    }
                }
            }
        }
    }

    return maior;
}

// Algoritmo Distância -- sem peso 
int Grafo::distancia(unsigned int start, const string& outputFile, bool write_tree, unsigned int end){
    if (grafo_peso){
        cerr << "Essa função é apenas para grafos sem peso" << endl;
        return 0;
    }
    int level;
    BFS(start, outputFile, write_tree,end);
    level = nivel[end];
    return level;
}

// Algoritmo BFS -- sem peso 
void Grafo::BFS(unsigned int s, const string& outputFile, bool write_tree, unsigned int e) {
    if (grafo_peso){
        cerr << "Essa função é apenas para grafos sem peso" << endl;
        return;
    }
    visitado.assign(vertices + 1, false);
    pai.assign(vertices + 1, -1);
    nivel.assign(vertices + 1, -1);

    queue<unsigned int> Q;
    
    visitado[s] = true;
    nivel[s] = 0;
    Q.push(s);

    while (!Q.empty()) {
        if (e!= 0 and visitado[e]){
            break;
        }
        unsigned int v = Q.front();
        Q.pop();

        const vector<unsigned int>& vizinhos = usaMatriz ? getVizinhosMatriz(v) : listAdj[v];
        for (unsigned int w : vizinhos) {
            if (!visitado[w]) {
                visitado[w] = true;
                pai[w] = v;      // Armazena o pai de w
                nivel[w] = nivel[v] + 1; // Define o nível de w
                Q.push(w);
            }
        }
    }
    if (write_tree){
        Write_file_busca(pai, nivel, outputFile, s);
    }
}

// Algoritmo DFS -- sem peso 
void Grafo::DFS(unsigned int s, const string& outputFile, bool write_tree) {
    if (grafo_peso){
        cerr << "Essa função é apenas para grafos sem peso" << endl;
        return;
    }
    visitado.assign(vertices + 1, false);
    pai.assign(vertices + 1, -1);
    nivel.assign(vertices + 1, -1);

    stack<unsigned int> pilha;

    pilha.push(s);
    nivel[s] = 0;

    while (!pilha.empty()) {
        unsigned int v = pilha.top();
        pilha.pop();

        if (!visitado[v]) {
            visitado[v] = true;

            // Para cada vizinho do vértice v
            const vector<unsigned int>& vizinhos = usaMatriz ? getVizinhosMatriz(v) : listAdj[v];
            for (unsigned int w : vizinhos) {
                if (!visitado[w]) {
                    pilha.push(w);
                    pai[w] = v;           // Define o pai de w
                    nivel[w] = nivel[v] + 1; // Define o nível de w
                }
            }
        }
    }
    if (write_tree){
        Write_file_busca(pai, nivel, outputFile, s);
    }
}

// Algoritmo Componentes Conexas
map<unsigned int, vector<vector<unsigned int>>, greater<unsigned int>> Grafo::ComponentesConexas() {   
    // Mapa onde a chave é o tamanho da componente e os valores são os vértices da componente
    map<unsigned int, vector<vector<unsigned int>>, greater<unsigned int>> componentesConexas;
    vector<bool> visitado(vertices + 1, false); // Vetor para marcar vértices visitados
    queue<unsigned int> Q; // Fila para a BFS

    // Percorre todos os vértices do grafo
    for (unsigned int i = 1; i <= vertices; ++i) {
        if (!visitado[i]) {
            // Se o vértice ainda não foi visitado, achamos uma nova componente conexa
            vector<unsigned int> componenteAtual; // Armazena os vértices da componente atual
            n_CompConexa ++;
            // Executa a BFS a partir do vértice 'i'
            Q.push(i);
            visitado[i] = true;

            while (!Q.empty()) {
                unsigned int v = Q.front();
                Q.pop();

                componenteAtual.push_back(v); // Adiciona o vértice à componente atual

                const vector<unsigned int>& vizinhos = usaMatriz ? getVizinhosMatriz(v) : listAdj[v];
                for (unsigned int w : vizinhos) {
                    if (!visitado[w]) {
                        visitado[w] = true;
                        Q.push(w);
                    }
                }
            }

            // Adiciona a componente atual ao mapa, usando o tamanho da componente como chave
            unsigned int tamanho = componenteAtual.size();
            componentesConexas[tamanho].push_back(componenteAtual); 
        }
    }

    return componentesConexas; // Retorna o mapa de componentes
}

// Imprimir Componentes Conexas
void Grafo::imprimirComponentesConexas(const map<unsigned int, vector<vector<unsigned int>>, std::greater<unsigned int>>& componentesConexas) {
    if (componentesConexas.empty()) {
        cout << "Nenhuma componente conexa encontrada." << endl;
        return;
    }

    // Itera sobre o map onde a chave é o tamanho da componente
    for (const auto& par : componentesConexas) {
        long long tamanho = par.first; // A chave (tamanho da componente)
        const vector<vector<unsigned int>>& componentes = par.second; // O vetor de componentes conexas de mesmo tamanho

        cout << "Componentes de tamanho " << tamanho << ":" << endl;

        // Itera sobre cada componente conexa (um vetor de vértices)
        for (const auto& componente : componentes) {
            cout << "Vértices: ";
            for (unsigned int vertice : componente) {
                cout << vertice << " "; // Imprime cada vértice da componente
            }
            cout << endl;
        }

        cout << endl; // Nova linha entre as componentes de mesmo tamanho
    }
}


// Algoritmo de Dijkstra que calcula a distância de um vértice para todos os outros
// Retorna o vetor de distâncias e opcionalmente escreve a árvore geradora em arquivo
vector<float> Grafo::algoritmo_Dijkstra(unsigned int s, const string& outputFile, bool write_tree, bool Heap) {
    // Inicializando o vetor de distâncias com infinito
    dist.assign(vertices + 1, numeric_limits<float>::infinity());
    pai.assign(vertices + 1, -1);
    vector<bool> visitado(vertices + 1, false);
    if (!grafo_peso){
        cerr << "Essa função é apenas para grafos com peso" << endl;
        return dist;
    }
    
    // A distância do vértice inicial para ele mesmo é 0
    dist[s] = 0;

    if (Heap) {
        // Usando priority_queue (min-heap) para o caso Heap == true
        // A fila armazena pares (distância, vértice)
        priority_queue<pair<float, unsigned int>, vector<pair<float, unsigned int>>, greater<pair<float, unsigned int>>> pq;
        pq.push({0, s});

        while (!pq.empty()) {
            unsigned int u = pq.top().second;
            pq.pop();
            if (!visitado[u]){
                // Para cada vizinho v de u
                visitado[u] = true;
                const vector<tuple<unsigned int,float>>& vizinhos = usaMatriz ? getVizinhosMatriz_peso(u) : listAdj_peso[u];
                for (const auto& vizinho : vizinhos) {
                    unsigned int v = get<0>(vizinho);
                    float peso = get<1>(vizinho);
                    if (peso < 0){
                        cerr << "A biblioteca ainda nao implementa caminhos minimos com pesos negativos." << endl;
                        return dist;
                    }
                    if (dist[v] > dist[u] + peso) {
                        dist[v] = dist[u] + peso;  // Atualiza a distância de v
                        pai[v] = u;  // Armazena o pai para reconstruir o caminho
                        pq.push({dist[v], v});  // Adiciona v à fila de prioridade
                    }
                }
            }
        }
    } else {
        // Usando vetor para o caso Heap == false
        
        for (unsigned int i = 0; i < vertices; ++i) {
            // Seleciona o vértice u não visitado com a menor distância
            float menorDist = numeric_limits<float>::infinity();
            unsigned int u = -1;

            for (unsigned int v = 1; v <= vertices; ++v) {
                if (!visitado[v] && dist[v] < menorDist) {
                    menorDist = dist[v];
                    u = v;
                }
            }

            // Marca u como visitado
            visitado[u] = true;

            // Para cada vizinho v de u
            const vector<tuple<unsigned int,float>>& vizinhos = usaMatriz ? getVizinhosMatriz_peso(u) : listAdj_peso[u];

            for (const auto& vizinho : vizinhos) {
                unsigned int v = get<0>(vizinho);
                float peso = get<1>(vizinho);
                if (peso < 0){
                    cerr << "A biblioteca ainda nao implementa caminhos minimos com pesos negativos." << endl;
                    return dist;
                }
                if (dist[v] > dist[u] + peso) {
                    dist[v] = dist[u] + peso;  // Atualiza a distância de v
                    pai[v] = u;  // Armazena o pai para reconstruir o caminho
                }
            }
        }
    }

    // Se write_tree for true, grava a árvore geradora no arquivo de saída
    if (write_tree) {
        Write_file_busca_peso(pai, dist, outputFile, s);
    }
    return dist;
}

// funçao distancia -- entra com booleano de heap, vertice inicial e final, retorna a distancia entre esses 2 pontos
float Grafo::distancia_peso(unsigned int start, unsigned int end, bool Heap) {
    // Calcula a distância de 'start' para todos os outros vértices
    vector<float> distancias = algoritmo_Dijkstra(start, "", false, Heap);

    // Retorna a distância do vértice 'start' para o vértice 'end'
    return distancias[end];
}
// funçao caminho minimos -- entra com booleano de heap, vertice inicial e final, retorna o caminho mínimo entre esses 2 pontos
vector<unsigned int> Grafo::caminho_minimo_peso(unsigned int start, unsigned int end, bool Heap) {
    // Calcula a distância de 'start' para todos os outros vértices e constrói a árvore geradora
    algoritmo_Dijkstra(start, "", false, Heap);

    // Vetor para armazenar o caminho mínimo
    vector<unsigned int> caminho;

    // Reconstrói o caminho mínimo a partir do vetor de pais
    for (unsigned int v = end; v != -1; v = pai[v]) {
        caminho.push_back(v);
    }

    // O caminho está ao contrário, então invertemos
    reverse(caminho.begin(), caminho.end());

    // Verifica se o caminho encontrado é válido (se o último vértice é 'start')
    if (caminho.front() != start) {
        return {}; // Retorna um vetor vazio se não houver caminho
    }

    return caminho;
}


