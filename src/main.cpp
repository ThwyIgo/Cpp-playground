#include <iostream>
#include <string>
#include "graph.hpp"

int main(void)
{
    using namespace std;
    cout.setf(ios::boolalpha);

    Graph<string, unsigned int> meuGrafo = {
        {"Udia", {
                {"Udia", 0},
                {"Araguari", 30},
                {"Beraba", 60},
                {"Cuiaba", 1200},
                {"Brasilia", 700}
            }
        },
        {"Araguari", {
                {"Udia", 30},
                {"Beraba", 40},
                {"Brasilia", 100}
            }
        },
        {"Brasilia", {
                {"Cuiaba", 400},
                {"Cavalcante", 100}
            }
        }
    };

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
    meuGrafo.print();

    cout << "\nChanging an element (doesn't change the vertex 'globally')\n";
    meuGrafo.getEdgePtr("Brasilia", "Cuiaba")->first = "Belzonte";
    meuGrafo.print();

    cout << "\nIf you want to change all vertices easily, use a pointer as the type T of the Graph\n";
}
