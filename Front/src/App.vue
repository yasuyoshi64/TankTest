<script setup>
import { ref, watch, computed, onMounted } from 'vue'
import axios from 'axios'
import ControlBall from '@/components/ControlBall.vue'

// data
const ipAddress = ref("")
const speedData = ref(0)        // 速度 -100～100
const steeringData = ref(0)     // ハンドル -100～100
const steeringTrimData = ref(0) // ハンドルトリム -100～100

const speed = ref(null)
const steering = ref(null)
const steeringTrim = ref(null)
// WebSocket
var ws = null
const isWSConnected = ref(false)
const wsMessage = ref("")

// WebSocketの接続状態を1秒間隔でチェック
setInterval(checkWebSocketState, 1000)

function checkWebSocketState() {
  if (ws === null)
    return;
  switch (ws.readyState) {
    case WebSocket.CONNECTING:
      wsMessage.value = "接続中"
      break;
    case WebSocket.OPEN:
      wsMessage.value = "接続済"
      break;
    case WebSocket.CLOSING:
      wsMessage.value = "切断中"
      break;
    case WebSocket.CLOSED:
      wsMessage.value = "切断"
      isWSConnected.value = false
      break;
  }
}

// 操作
watch(speedData, async(newValue, oldValue) => {
  console.log(`speedData=${speedData.value}`)
  ws_send()
})
watch(steeringData, async(newValue, oldValue) => {
  console.log(`steeringData=${steeringData.value}`)
  ws_send()
})

// "speed=100,steering=0"
// speed=-100～100
// steering=-100～100  (speed=0の場合は超信地旋回)
function ws_send() {
  try {
    const steeringCalc = speedData.value != 0 || steeringData.value != 0 ? steeringData.value + steeringTrimData.value : steeringData.value
    const value = `speed=${speedData.value},steering=${steeringCalc}`
    if (isWSConnected.value === true) {
      // [STX][value][ETX][checksum]に変換
      const encoder = new TextEncoder()
      const encoded = encoder.encode(value)
      const val = encoded.buffer
      const dest = new Uint8Array(val.byteLength + 3)
      dest[0] = 0x02
      dest.set(encoded, 1)
      dest[dest.length - 2] = 0x03
      const checksum = encoded.reduce((acc, byte) => acc ^ byte, 0)
      dest[dest.length - 1] = checksum
      console.log(`${value}, checksum=${checksum}`)
      ws.send(dest)
    }
  } catch (error) {
    console.log("エラー: サーバーDisconnect", error)
    ipAddress.value = ""
    isWSConnected.value = false
  }
}

onMounted(() => {
  // // フリックでの画面スクロールを禁止する
  // document.body.addEventListener('touchmove', function(event) {
  //   event.preventDefault();   // デフォルト動作の禁止
  // }, { passive: false})
  // // マウスホイールによる画面スクロールを禁止する
  // document.body.addEventListener('wheel', function(event) {
  //   event.preventDefault();   // デフォルト動作の禁止
  // }, { passive: false})

  // データ取得
  axios
    .get(encodeURI("./API/get_data"))
    .then(response => {
      if (response.headers["content-type"] == "application/json") {
        ipAddress.value = response.data.ip_address
      }
    })
    .catch(error => {
      log.console("./API/get_data request error")
    })
  // WebSocket接続
  const url = "ws://" + window.location.host + "/ws"
  ws = new WebSocket(url)
  ws.onopen = (event) => {
    console.log("WS : サーバーConnect")
    isWSConnected.value = true
//    ws_send()
  }
  ws.onclose = (event) => {
    console.log("WS : サーバーDisconnect", event)
    ipAddress.value = ""
    isWSConnected.value = false
  }
  ws.onmessage = (event) => {
    console.log("WS : サーバーから受信 : " + event.data)
  }
  ws.onerror = (event) => {
    console.error("WS : エラー", event)
  }
})

function save() {
  axios
    .post(encodeURI("./API/save"), "", { headers: { 'Content-Type': 'application/json'}})
}

const controlWidth = computed(() => {
  return parseInt((window.innerWidth / 2))
})

const controlHeight = computed(() => {
  return parseInt(window.innerHeight)
})

</script>

<template>
  <main>
    <div style="position: fixed;">
      connected: {{ isWSConnected }}<span v-if="ipAddress !== ''">, ip address: {{ ipAddress }}</span>
      <span v-if="wsMessage !== ''">, {{ wsMessage }}</span>
    </div>
    <div class="flex-container">
      <div class="flex-item">
        <ControlBall direction="vertical" :min="-100" :max="100" height="100%" :width="controlHeight" v-model="speedData"/>
      </div>
      <div class="flex-item">
        <div style="height: 25vh;">
        </div>
        <div style="height: 40vh;">
          <ControlBall direction="horizontal" :min="-100" :max="100" :width="controlWidth" height="110" v-model="steeringData"/><br/>
        </div>
        <div style="height: 35vh; position: relative; text-align: center;">
          <div style="position: absolute; bottom: 5em;">
            Steering Trim<br/>
            <ControlBall direction="horizontal" :min="-100" :max="100" type="fixed" :width="controlWidth" height="50" :ballweight="20" :lineWidth="50" v-model="steeringTrimData"/>
          </div>
        </div>
      </div>
    </div>
  </main>
</template>

<style scoped>
html, body {
  height: 100%;
  min-height: 100vh;
  margin: 0;
  padding: 0;
  overflow: hidden;     /* スクロールバーを非表示、スクロールを禁止 */
}

.flex-container {
  height: 95vh;
  width: 100vw;
  position: fixed;
  top: 0;
  left: 0;
  display: flex;
  justify-content: space-around;
  align-items: center;
}

.flax-item {
  height: 95vh;
  width: 50vw;
}

</style>
