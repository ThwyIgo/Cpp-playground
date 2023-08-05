#include <iostream>
#include <string>
#include "graphs.hpp"

int main(void) {
    using namespace std;
    cout.setf(ios::boolalpha);

    WeightedDigraph<string, unsigned int> meuGrafo = {
        {"Udia",
         {{"Udia", 0},
          {"Araguari", 30},
          {"Beraba", 60},
          {"Cuiaba", 1200},
          {"Brasilia", 700}}},
        {"Araguari", {{"Udia", 30}, {"Beraba", 40}, {"Brasilia", 100}}},
        {"Brasilia", {{"Cuiaba", 400}, {"Cavalcante", 100}}}};

    meuGrafo.addVert("Maceio");
    cout << meuGrafo.contains("Maceio") << '\n';
    cout << meuGrafo.contains("Araguari", "Udia") << '\n';
    meuGrafo.print();
    cout << meuGrafo.size() << '\n';
    meuGrafo.removeVert("Maceio");

    auto udiaShortestPathTree = meuGrafo.dijkstra("Udia");
    cout << "Distances from Udia:\n";
    meuGrafo.printShortestDistances(udiaShortestPathTree);

    cout << "\nPath to Brasilia\n";
    meuGrafo.printPrevPath(udiaShortestPathTree, "Brasilia");

    cout << "\nRemoving Beraba from Udia and adding Araguari to Beraba\n";
    meuGrafo.removeEdge("Udia", "Beraba");
    meuGrafo.addEdge("Beraba", "Araguari", 40, true);
    meuGrafo.addEdge("Beraba", "Pertinho");
    meuGrafo.print();

    cout << "\nChanging an element:\n";
    meuGrafo.modifyVertex("Brasilia", "Belzonte");
    meuGrafo.modifyEdgeWeight("Udia", "Belzonte", 531);
    meuGrafo.print();
    cout << meuGrafo.contains("Brasilia") << '\n';

    cout << "Add 'city_' to every vertice\n";
    for (auto& v : meuGrafo)
        v = "city_" + v;

    // Print vertices with a for-loop
    for (auto v : meuGrafo)
        cout << v << ", ";
    cout << '\n';

    undigraph<int> undirectedG;

    undirectedG.addEdge(0, 5);
    undirectedG.addEdge(0, 4);
    undirectedG.addEdge(4, 2);

    cout << "sera" << undirectedG.isBipartite() << endl;

    cout << undirectedG.contains(4, 2) << endl;
    cout << undirectedG.contains(4, 0) << endl;
    undirectedG.removeEdge(0, 4);
    cout << undirectedG.contains(4, 0) << undirectedG.contains(0, 4) << endl;
}
