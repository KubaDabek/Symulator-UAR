#define _CRT_SECURE_NO_WARNINGS
#include "klasy.h"

using namespace std;

int main() {
    std::vector<double> a = { -0.4 };
    std::vector<double> b = { 0.6 };
    ARXModel arxModel(a, b, 1);

    PIDController pid(1.0, 0.1, 0.05);
    pid.ustawLimity(-1.0, 1.0);

    arxModel.setPIDController(&pid);
    pid.setARXModel(&arxModel);

    double wartoscZadana = 1.0;
    double wartoscProcesu = 0.0;

    std::vector<double> wynikiSymulacji;
    wynikiSymulacji.reserve(200);

    for (int i = 0; i < 200; ++i) {
        double sygnalKontrolny = pid.oblicz(wartoscZadana, wartoscProcesu);
        wartoscProcesu = arxModel.krok(sygnalKontrolny);
        wynikiSymulacji.push_back(wartoscProcesu);
    }

    zapiszSymulacjeTekst("symulacja.txt", arxModel, pid, wynikiSymulacji);
    zapiszSymulacjeBin("symulacja.bin", arxModel, pid, wynikiSymulacji);

    ARXModel nowyArx;
    PIDController nowyPid;
    std::vector<double> wczytaneWyniki;

    wczytajSymulacjeTekst("symulacja.txt", nowyArx, nowyPid, wczytaneWyniki);
    std::cout << "Wczytano z symulacja.txt:\n";
    std::cout << "Liczba kroków: " << wczytaneWyniki.size() << "\n";
    std::cout << "Ostatnia wartość z symulacja.txt: " << wczytaneWyniki.back() << "\n";

    wczytaneWyniki.clear();
    wczytajSymulacjeBin("symulacja.bin", nowyArx, nowyPid, wczytaneWyniki);
    std::cout << "Wczytano z symulacja.bin:\n";
    std::cout << "Liczba kroków: " << wczytaneWyniki.size() << "\n";
    std::cout << "Ostatnia wartość z symulacja.bin: " << wczytaneWyniki.back() << "\n";

    return 0;
}
