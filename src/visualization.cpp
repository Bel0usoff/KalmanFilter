#include "inc.hpp"

void plot_gps_trajectory(const std::string& csv_file,const std :: string& filename) {
    std::ofstream script("plot_gps.gp");
    
    script << "set terminal pngcairo size 800,600 enhanced font 'Arial,12'\n"
           << "set output '"<<filename<<".png'\n"
           << "set title 'Траектория движения (GPS)'\n"
           << "set xlabel 'Долгота (Longitude)'\n"
           << "set ylabel 'Широта (Latitude)'\n"
           << "set grid\n"
           << "set datafile separator comma\n"
           << "set key autotitle columnhead\n\n"
           << "plot '" << csv_file << "' \\\n"
           << "     using 3:2 with linespoints lc rgb 'red' title 'Траектория'\n"
           << "set output\n";
    
    script.close();
   

    system("gnuplot plot_gps.gp");
    std::cout << "График сохранен в "<<filename<<".png" << std::endl;
}

void showInteractivePlot(const std::string& csvFile) {
    // Формируем команду для Gnuplot
    std::string command = "gnuplot -persist -e \"";
    command += "set title 'GPS Trajectory'; ";
    command += "set xlabel 'Longitude'; ";
    command += "set ylabel 'Latitude'; ";
    command += "set grid; ";
    command += "set datafile separator comma; ";
    command += "plot '" + csvFile + "' using 3:2 with linespoints lc rgb 'red' title 'Trajectory'; ";
    command += "\"";
    
    system(command.c_str());
}

void createOverlayPlot(const std::string& csvFile_source,const std::string& csvFile_filter) {
    std::ofstream script("overlay_plot.gp");
    
    script << "set terminal pngcairo size 800,600 enhanced font 'Arial,12'\n"
           << "set output 'overlay.png'\n"
           << "set title 'Траектория движения (GPS)'\n"
           << "set xlabel 'Долгота'\n"
           << "set ylabel 'Широта'\n"
           << "set grid\n"
           << "set datafile separator comma\n\n"  // Явно указываем разделитель CSV
           
           << "plot \\\n"  // Обратите внимание на один обратный слэш
           << "  '" << csvFile_source << "' using 3:2 with linespoints lc rgb 'red' title 'Траектория с датчика', \\\n"
           << "  '" << csvFile_filter << "' using 3:2 with linespoints lc rgb 'blue' title 'Траектория после фильтра'\n"
           << "unset output\n";  // Лучше использовать unset вместо set output

    script.close();

    system("gnuplot overlay_plot.gp");
}