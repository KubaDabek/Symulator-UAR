#define _CRT_SECURE_NO_WARNINGS
#pragma once
#include <vector>
#include <queue>
#include <random>
#include <iostream>
#include <algorithm>

class ARXModel {
private:
    std::vector<double> A;       // wspolczynnik wyjscia
    std::vector<double> B;       // wspolczynnik wejscia
    std::queue<double> wejscieBufor;
    std::queue<double> wyjscieBufor;
    int opoznienie;
    double szum;
    std::default_random_engine generator;
    std::normal_distribution<double> dystrybucja;  // losowe wartosci zaklocenia

public:
    ARXModel(const std::vector<double>& a, const std::vector<double>& b, int k, double szum = 0.01)
        : A(a), B(b), opoznienie(k), szum(szum), dystrybucja(0.0, szum) {
        // bufory dla współczynników
        wejscieBufor = std::queue<double>(std::deque<double>(B.size(), 0.0));
        wyjscieBufor = std::queue<double>(std::deque<double>(A.size(), 0.0));
    }

    // krok w symulacji
    double krok(double input) {
        wejscieBufor.push(input);
        if (wejscieBufor.size() > B.size()) {
            wejscieBufor.pop();
        }

        // wartosc wyjsciowa
        double wyjscie = 0.0;

        int i = 0;
        for (const auto& b : B) {
            wyjscie += b * wejscieBufor.front();
            wejscieBufor.pop();
            wejscieBufor.push(wyjscie);
        }

        i = 0;
        for (const auto& a : A) {
            wyjscie -= a * wyjscieBufor.front();
            wyjscieBufor.pop();
            wyjscieBufor.push(wyjscie);  // ustawienie ponownie w kolejce
        }

        // zaklocenie
        wyjscie += dystrybucja(generator);

        // aktualizacja buforu wyjsciowego
        wyjscieBufor.push(wyjscie);
        if (wyjscieBufor.size() > A.size()) {
            wyjscieBufor.pop();
        }

        return wyjscie;
    }
};

template <typename T>
T filtr(T wartosc, T dolny, T gorny) {
    return std::max(dolny, std::min(wartosc, gorny));
}

class PIDController {
private:
    double kp, ki, kd;                  // wspolczynniki PID
    double calka, bladPoprzedzajacy;
    double dolnyLimit, gornyLimit;      // limity przeciwnasyceniowe
    bool flagaPrzeciwNasyceniowa;       // flaga wlaczajaca przeciwnasycenie

public:
    // Konstruktor ustawiajacy poczatkowe wartosci wspolczynnikow PID i ograniczen wyjscia
    PIDController(double kp, double ki, double kd, double dolnyLimit = -1.0, double gornyLimit = 1.0)
        : kp(kp), ki(ki), kd(kd), dolnyLimit(dolnyLimit), gornyLimit(gornyLimit), calka(0.0), bladPoprzedzajacy(0.0),
        flagaPrzeciwNasyceniowa(true) {}

    // limity przeciwnasyceniowe
    void ustawLimity(double nizszy, double wyzszy) {
        dolnyLimit = nizszy;
        gornyLimit = wyzszy;
    }

    // resetowanie czlonow calkujacego i rozniczkujacego
    void reset() {
        calka = 0.0;
        bladPoprzedzajacy = 0.0;
    }

    // obliczenie wartosc regulacji na podstawie uchybu
    double oblicz(double ustawWartosc, double wartoscProcesu) {
        double blad = ustawWartosc - wartoscProcesu;
        calka += blad;
        double pochodna = blad - bladPoprzedzajacy;
        bladPoprzedzajacy = blad;

        double wyjscie = kp * blad + ki * calka + kd * pochodna;

        // przeciwnasyceniowy filtr
        if (flagaPrzeciwNasyceniowa) {
            wyjscie = filtr(wyjscie, dolnyLimit, gornyLimit);
        }

        return wyjscie;
    }
};


