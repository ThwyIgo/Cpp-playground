#include <iostream>
#include <string>
#include "graph.hpp"

int main(void)
{
    using namespace std;

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

    auto udiaShortestPathTree = meuGrafo.dijkstra("Udia");
    cout << "Distances from Udia:\n";
    meuGrafo.printShortestDistances(udiaShortestPathTree);

    cout << "\nPath to Brasilia\n";
    meuGrafo.printPrevPath(udiaShortestPathTree, "Brasilia");

    cout << "\nRemoving Beraba from Udia and adding Araguari to Beraba\n";
    meuGrafo.removeEdge("Udia", "Beraba");
    meuGrafo.addEdge("Beraba", "Araguari", 40, true);
    meuGrafo.print();

    cout << "\nChanging an element\n";
    meuGrafo.getEdgePtr("Brasilia", "Cuiaba")->first = "Belzonte";
    meuGrafo.print();
}
