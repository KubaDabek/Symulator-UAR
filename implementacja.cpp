#define _CRT_SECURE_NO_WARNINGS
#include "klasy.h"

using namespace std;

int main() {
    // wspolczynniki ARX
    std::vector<double> a = { -0.4 };
    std::vector<double> b = { 0.6 };
    ARXModel arxModel(a, b, 1);

    // ustawienia regulatora PID
    PIDController pid(1.0, 0.1, 0.05);
    pid.ustawLimity(-1.0, 1.0);

    // wartosc zadana
    double wartoscZadana = 1.0;
    double wartoscProcesu = 0.0;

    // Symulacja
    for (int i = 0; i < 100; ++i) {
        double sygnalKontrolny = pid.oblicz(wartoscZadana, wartoscProcesu);
        wartoscProcesu = arxModel.krok(sygnalKontrolny);
        std::cout << "Krok: " << i
            << " -> Sterowanie: " << sygnalKontrolny
            << " Wyjscie: " << wartoscProcesu
            << std::endl;
    }
    return 0;
}
