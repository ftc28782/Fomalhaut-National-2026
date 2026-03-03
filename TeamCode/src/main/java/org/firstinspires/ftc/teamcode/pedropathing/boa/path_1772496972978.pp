{
  "startPoint": {
    "x": 56,
    "y": 8,
    "heading": "linear",
    "startDeg": 90,
    "endDeg": 180,
    "locked": false
  },
  "lines": [
    {
      "id": "line-4ddoqdq2e84",
      "name": "Intake1",
      "endPoint": {
        "x": 17,
        "y": 35.13078470824951,
        "heading": "linear",
        "startDeg": 90,
        "endDeg": 180
      },
      "controlPoints": [
        {
          "x": 60.26559356136822,
          "y": 38.24547283702213
        }
      ],
      "color": "#7DB9BD",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mm9ui66v-rf2yza",
      "name": "toShoot2",
      "endPoint": {
        "x": 59.0281690140845,
        "y": 13.076458752515094,
        "heading": "linear",
        "reverse": false,
        "startDeg": 180,
        "endDeg": 120
      },
      "controlPoints": [],
      "color": "#98ADAC",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mm9uja9j-jlvxpg",
      "name": "Intake2",
      "endPoint": {
        "x": 9.561368209255534,
        "y": 15.35613682092559,
        "heading": "linear",
        "reverse": false,
        "startDeg": 120,
        "endDeg": 180
      },
      "controlPoints": [
        {
          "x": 36.914486921529175,
          "y": 12.96277665995977
        }
      ],
      "color": "#D55C77",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mm9umxmg-8ng450",
      "name": "2Intake2",
      "endPoint": {
        "x": 10.430583501006037,
        "y": 9.617706237424573,
        "heading": "linear",
        "reverse": false,
        "startDeg": 180,
        "endDeg": 180
      },
      "controlPoints": [
        {
          "x": 24.440643863179076,
          "y": 11.85110663983905
        }
      ],
      "color": "#D68988",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mm9uosnt-m78ac8",
      "name": "Shoot3",
      "endPoint": {
        "x": 56.736418511066404,
        "y": 12.645875251509057,
        "heading": "linear",
        "reverse": false,
        "startDeg": 180,
        "endDeg": 120
      },
      "controlPoints": [],
      "color": "#5CCD98",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    }
  ],
  "shapes": [
    {
      "id": "triangle-1",
      "name": "Red Goal",
      "vertices": [
        {
          "x": 144,
          "y": 70
        },
        {
          "x": 144,
          "y": 144
        },
        {
          "x": 120,
          "y": 144
        },
        {
          "x": 138,
          "y": 119
        },
        {
          "x": 138,
          "y": 70
        }
      ],
      "color": "#dc2626",
      "fillColor": "#ff6b6b"
    },
    {
      "id": "triangle-2",
      "name": "Blue Goal",
      "vertices": [
        {
          "x": 6,
          "y": 119
        },
        {
          "x": 25,
          "y": 144
        },
        {
          "x": 0,
          "y": 144
        },
        {
          "x": 0,
          "y": 70
        },
        {
          "x": 7,
          "y": 70
        }
      ],
      "color": "#2563eb",
      "fillColor": "#60a5fa"
    }
  ],
  "sequence": [
    {
      "kind": "path",
      "lineId": "line-4ddoqdq2e84"
    },
    {
      "kind": "path",
      "lineId": "mm9ui66v-rf2yza"
    },
    {
      "kind": "path",
      "lineId": "mm9uja9j-jlvxpg"
    },
    {
      "kind": "path",
      "lineId": "mm9umxmg-8ng450"
    },
    {
      "kind": "path",
      "lineId": "mm9uosnt-m78ac8"
    }
  ],
  "settings": {
    "xVelocity": 150,
    "yVelocity": 150,
    "aVelocity": 31.41592653589793,
    "kFriction": 0.1,
    "rWidth": 16,
    "rHeight": 18,
    "safetyMargin": 1,
    "maxVelocity": 100,
    "maxAcceleration": 100,
    "maxDeceleration": 100,
    "fieldMap": "decode.webp",
    "robotImage": "/robot.png",
    "theme": "auto",
    "showGhostPaths": false,
    "showOnionLayers": false,
    "onionLayerSpacing": 3,
    "onionColor": "#dc2626",
    "onionNextPointOnly": false
  },
  "version": "1.2.1",
  "timestamp": "2026-03-03T00:16:15.233Z"
}