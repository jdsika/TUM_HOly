#!/bin/bash

echo "setting compliance slope, margin and punch..."
# removed the arms
for m in HAA HR HFE KFE AFE AR; do
  for s in L R; do
    echo -n $s'_'$m', '

    rosservice call '/'$s'_'$m'_controller/set_speed' 5.3 > /dev/null &
    rosservice call '/'$s'_'$m'_controller/torque_enable' 1 > /dev/null &
    rosservice call '/'$s'_'$m'_controller/set_compliance_slope' 12 > /dev/null &
    rosservice call '/'$s'_'$m'_controller/set_compliance_margin' 0 > /dev/null &
    rosservice call '/'$s'_'$m'_controller/set_compliance_punch' 0 > /dev/null &

  done
done

wait
echo -e "\n... done."
