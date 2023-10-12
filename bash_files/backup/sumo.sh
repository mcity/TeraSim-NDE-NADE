for i in 0 1 2 3 4 5 6 7 8 9; do
    sumo -c maps/Mcity/mcity.sumocfg --fcd-output test/fcd_all.xml --fcd-output.acceleration
    python visualize.py --path test --file ${i}
done