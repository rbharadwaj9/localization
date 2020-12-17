DATAFILE=${1}

echo "Generating Motion Data"
python action_groundtruth_gen/action_groundtruth_gen.py ${DATAFILE}

echo "Generating Sensor Noise"
python sensor_gen/sensor_model.py ${DATAFILE}
