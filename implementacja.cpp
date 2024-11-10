#define _CRT_SECURE_NO_WARNINGS
#include "klasy.h"

using namespace std;

int main() {
    // przykladowe wspó³czynniki ARX
    std::vector<double> a = { -0.4 };
    std::vector<double> b = { 0.6 };
    ARXModel arxModel(a, b, 1);

    // ustawienia regulatora PID
    PIDController pid(1.0, 0.1, 0.05);
    pid.ustawLimity(-1.0, 1.0);

    // symulacja sterowania
    double p = 1.0; // wartoœæ zadana
    double q = 0.0;
    for (int i = 0; i < 10; ++i) {
        double sygnalKontrolny = pid.oblicz(p, q);
        q = arxModel.krok(sygnalKontrolny);
        std::cout << "Czas: " << i << " -> Sterowanie: " << sygnalKontrolny << " Wyjscie: " << q << std::endl;
    }
    
    return 0;
}
