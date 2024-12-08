#define _CRT_SECURE_NO_WARNINGS
#pragma once
#include <vector>
#include <queue>
#include <random>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <string>
#include <deque>

class ARXModel {
private:
    std::vector<double> A;
    std::vector<double> B;
    std::queue<double> wejscieBufor;
    std::queue<double> wyjscieBufor;
    int opoznienie;
    double szum;
    std::default_random_engine generator;
    std::normal_distribution<double> dystrybucja;
    PIDController* pidController;

public:
    ARXModel(const std::vector<double>& a, const std::vector<double>& b, int k, double szum = 0.01)
        : A(a), B(b), opoznienie(k), szum(szum), dystrybucja(0.0, szum), pidController(nullptr)
    {
        wejscieBufor = std::queue<double>(std::deque<double>(B.size(), 0.0));
        wyjscieBufor = std::queue<double>(std::deque<double>(A.size(), 0.0));
    }

    ARXModel() : opoznienie(0), szum(0.0), dystrybucja(0.0, 0.01), pidController(nullptr) {}

    void setPIDController(PIDController* pid) {
        pidController = pid;
    }

    double krok(double input) {
        wejscieBufor.push(input);
        if (wejscieBufor.size() > B.size()) {
            wejscieBufor.pop();
        }

        double wyjscie = 0.0;
        {
            std::queue<double> p = wejscieBufor;
            for (auto b : B) {
                wyjscie += b * p.front();
                p.pop();
            }
        }

        {
            std::queue<double> w = wyjscieBufor;
            for (auto a : A) {
                wyjscie -= a * w.front();
                w.pop();
            }
        }

        wyjscie += dystrybucja(generator);

        wyjscieBufor.push(wyjscie);
        if (wyjscieBufor.size() > A.size()) {
            wyjscieBufor.pop();
        }

        return wyjscie;
    }

    void zapiszKonfiguracjeTekst(std::ostream& zapisz) const {
        zapisz << A.size() << " ";
        for (auto val : A) zapisz << val << " ";
        zapisz << B.size() << " ";
        for (auto val : B) zapisz << val << " ";
        zapisz << opoznienie << " " << szum << "\n";
    }

    void wczytajKonfiguracjeTekst(std::istream& wczytaj) {
        size_t aSize, bSize;
        wczytaj >> aSize;
        A.resize(aSize);
        for (size_t i = 0; i < aSize; ++i) wczytaj >> A[i];

        wczytaj >> bSize;
        B.resize(bSize);
        for (size_t i = 0; i < bSize; ++i) wczytaj >> B[i];

        wczytaj >> opoznienie >> szum;
        dystrybucja = std::normal_distribution<double>(0.0, szum);
        wejscieBufor = std::queue<double>(std::deque<double>(B.size(), 0.0));
        wyjscieBufor = std::queue<double>(std::deque<double>(A.size(), 0.0));
    }

    void zapiszKonfiguracjeBin(std::ostream& zapisz) const {
        size_t aSize = A.size();
        size_t bSize = B.size();
        zapisz.write((char*)&aSize, sizeof(aSize));
        zapisz.write((char*)A.data(), aSize * sizeof(double));
        zapisz.write((char*)&bSize, sizeof(bSize));
        zapisz.write((char*)B.data(), bSize * sizeof(double));
        zapisz.write((char*)&opoznienie, sizeof(opoznienie));
        zapisz.write((char*)&szum, sizeof(szum));
    }

    void wczytajKonfiguracjeBin(std::istream& wczytaj) {
        size_t aSize, bSize;
        wczytaj.read((char*)&aSize, sizeof(aSize));
        A.resize(aSize);
        wczytaj.read((char*)A.data(), aSize * sizeof(double));

        wczytaj.read((char*)&bSize, sizeof(bSize));
        B.resize(bSize);
        wczytaj.read((char*)B.data(), bSize * sizeof(double));

        wczytaj.read((char*)&opoznienie, sizeof(opoznienie));
        wczytaj.read((char*)&szum, sizeof(szum));
        dystrybucja = std::normal_distribution<double>(0.0, szum);
        wejscieBufor = std::queue<double>(std::deque<double>(B.size(), 0.0));
        wyjscieBufor = std::queue<double>(std::deque<double>(A.size(), 0.0));
    }
};

template <typename T>
T filtr(T wartosc, T dolny, T gorny) {
    return std::max(dolny, std::min(wartosc, gorny));
}

class PIDController {
private:
    double kp, ki, kd;
    double calka, bladPoprzedzajacy;
    double dolnyLimit, gornyLimit;
    bool flagaPrzeciwNasyceniowa;
    ARXModel* arxModel;

public:
    PIDController(double kp, double ki, double kd, double dolnyLimit = -1.0, double gornyLimit = 1.0)
        : kp(kp), ki(ki), kd(kd), dolnyLimit(dolnyLimit), gornyLimit(gornyLimit),
        calka(0.0), bladPoprzedzajacy(0.0), flagaPrzeciwNasyceniowa(true), arxModel(nullptr) {}

    PIDController() : kp(0.0), ki(0.0), kd(0.0), calka(0.0), bladPoprzedzajacy(0.0), dolnyLimit(-1.0), gornyLimit(1.0), flagaPrzeciwNasyceniowa(true), arxModel(nullptr) {}

    void setARXModel(ARXModel* arx) {
        arxModel = arx;
    }

    void ustawLimity(double nizszy, double wyzszy) {
        dolnyLimit = nizszy;
        gornyLimit = wyzszy;
    }

    void reset() {
        calka = 0.0;
        bladPoprzedzajacy = 0.0;
    }

    double oblicz(double ustawWartosc, double wartoscProcesu) {
        double blad = ustawWartosc - wartoscProcesu;
        calka += blad;
        double pochodna = blad - bladPoprzedzajacy;
        bladPoprzedzajacy = blad;

        double wyjscie = kp * blad + ki * calka + kd * pochodna;
        if (flagaPrzeciwNasyceniowa) {
            wyjscie = filtr(wyjscie, dolnyLimit, gornyLimit);
        }
        return wyjscie;
    }

    void zapiszKonfiguracjeTekst(std::ostream& zapisz) const {
        zapisz << kp << " " << ki << " " << kd << " "
            << calka << " " << bladPoprzedzajacy << " "
            << dolnyLimit << " " << gornyLimit << " "
            << (flagaPrzeciwNasyceniowa ? 1 : 0) << "\n";
    }

    void wczytajKonfiguracjeTekst(std::istream& wczytaj) {
        int flaga;
        wczytaj >> kp >> ki >> kd
            >> calka >> bladPoprzedzajacy
            >> dolnyLimit >> gornyLimit
            >> flaga;
        flagaPrzeciwNasyceniowa = (flaga == 1);
    }

    void zapiszKonfiguracjeBin(std::ostream& zapisz) const {
        zapisz.write((char*)&kp, sizeof(kp));
        zapisz.write((char*)&ki, sizeof(ki));
        zapisz.write((char*)&kd, sizeof(kd));
        zapisz.write((char*)&calka, sizeof(calka));
        zapisz.write((char*)&bladPoprzedzajacy, sizeof(bladPoprzedzajacy));
        zapisz.write((char*)&dolnyLimit, sizeof(dolnyLimit));
        zapisz.write((char*)&gornyLimit, sizeof(gornyLimit));
        int flaga = flagaPrzeciwNasyceniowa ? 1 : 0;
        zapisz.write((char*)&flaga, sizeof(flaga));
    }

    void wczytajKonfiguracjeBin(std::istream& wczytaj) {
        int flaga;
        wczytaj.read((char*)&kp, sizeof(kp));
        wczytaj.read((char*)&ki, sizeof(ki));
        wczytaj.read((char*)&kd, sizeof(kd));
        wczytaj.read((char*)&calka, sizeof(calka));
        wczytaj.read((char*)&bladPoprzedzajacy, sizeof(bladPoprzedzajacy));
        wczytaj.read((char*)&dolnyLimit, sizeof(dolnyLimit));
        wczytaj.read((char*)&gornyLimit, sizeof(gornyLimit));
        wczytaj.read((char*)&flaga, sizeof(flaga));
        flagaPrzeciwNasyceniowa = (flaga == 1);
    }
};

void zapiszSymulacjeTekst(const std::string& nazwaPliku, const ARXModel& arx, const PIDController& pid, const std::vector<double>& wyniki) {
    std::ofstream zapisz(nazwaPliku);
    arx.zapiszKonfiguracjeTekst(zapisz);
    pid.zapiszKonfiguracjeTekst(zapisz);

    zapisz << wyniki.size() << "\n";
    for (auto w : wyniki) {
        zapisz << w << "\n";
    }
}

void wczytajSymulacjeTekst(const std::string& nazwaPliku, ARXModel& arx, PIDController& pid, std::vector<double>& wyniki) {
    std::ifstream wczytaj(nazwaPliku);
    arx.wczytajKonfiguracjeTekst(wczytaj);
    pid.wczytajKonfiguracjeTekst(wczytaj);

    size_t rozmiar;
    wczytaj >> rozmiar;
    wyniki.resize(rozmiar);
    for (size_t i = 0; i < rozmiar; ++i) {
        wczytaj >> wyniki[i];
    }
}

void zapiszSymulacjeBin(const std::string& nazwaPliku, const ARXModel& arx, const PIDController& pid, const std::vector<double>& wyniki) {
    std::ofstream zapisz(nazwaPliku, std::ios::binary);
    arx.zapiszKonfiguracjeBin(zapisz);
    pid.zapiszKonfiguracjeBin(zapisz);

    size_t rozmiar = wyniki.size();
    zapisz.write((char*)&rozmiar, sizeof(rozmiar));
    zapisz.write((char*)wyniki.data(), rozmiar * sizeof(double));
}

void wczytajSymulacjeBin(const std::string& nazwaPliku, ARXModel& arx, PIDController& pid, std::vector<double>& wyniki) {
    std::ifstream wczytaj(nazwaPliku, std::ios::binary);
    arx.wczytajKonfiguracjeBin(wczytaj);
    pid.wczytajKonfiguracjeBin(wczytaj);

    size_t rozmiar;
    wczytaj.read((char*)&rozmiar, sizeof(rozmiar));
    wyniki.resize(rozmiar);
    wczytaj.read((char*)wyniki.data(), rozmiar * sizeof(double));
}
