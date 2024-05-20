<script setup>
import { ref, onMounted } from 'vue'

const props = defineProps({
    direction: String,   // "vertical" or "horizontal"
    ballweight: { type: Number, default: 50 },
    min: Number,
    max: Number,
    type: { type: String, default: 'variable' }, // "fixed" or "variable"
    width: { type: Number, default: 200 },
    lineWidth: { type: Number, default: 50 },
})
const model = defineModel()

const ControlBall = ref(null)
const data = ref({
    isTouch: false,
    identifier: null,
    center: { x: 0, y: 0 },
    model: 0,
    modelPix: 0,
    innerWidth: 0,
    innerHeight: 0,
})

onMounted(() => {
    window.addEventListener('resize', resize)
    resize()
})

function resize() {
    // 以下で強制的にコンポーネントのサイズを指定されたサイズに変更します。やらないと親のHTMLで指定されたサイズになる。
    if (props.direction === 'vertical') {
        ControlBall.value.height = props.width
    } else {
        ControlBall.value.width = props.width
    }
    ControlBall.value.width = ControlBall.value.clientWidth
    ControlBall.value.height = ControlBall.value.clientHeight
    data.value.center.x = parseInt(ControlBall.value.width / 2)
    data.value.center.y = parseInt(ControlBall.value.height / 2)
    draw()
}

function draw() {
    var height = ControlBall.value.offsetHeight
    var width = ControlBall.value.offsetWidth

    const ctx = ControlBall.value.getContext('2d')
    ctx.beginPath()
    ctx.clearRect(0, 0, width, height)
    ctx.stroke()

    // コントローラーのガイド線描画
    drawStick(ctx)

    // コントローラー円描画
    console.log(`model=${data.value.model}`)
    var val = Math.abs(data.value.model)
    var x = data.value.center.x
    var y = data.value.center.y
    if (props.direction === 'vertical') {
        var move = parseInt((height / 2) * (val / 100.0))
        if (data.value.model > 0) {
            y -= move
        } else {
            y += move
        }
        console.log(`move=${move}`)
    } else {
        var move = parseInt((width / 2) * (val / 100.0))
        if (data.value.model > 0) {
            x += move
        } else {
            x -= move
        }
        console.log(`move=${move}`)
    }
    ctx.beginPath()
    ctx.fillStyle = "#0000FF"
    ctx.strokeStyle = "#000055"
    ctx.lineWidth = props.ballweight / 5
    ctx.arc(x, y, props.ballweight, 0, Math.PI * 2, true)    
    ctx.fill()
    ctx.stroke()
}

function drawStick(ctx) {
    ctx.beginPath()
    ctx.strokeStyle = "#FFFFFF"
    ctx.lineWidth = props.ballweight / 5
    ctx.lineCap = "round"
    if (props.direction === 'vertical') {
        const center = ControlBall.value.offsetWidth / 2
        ctx.moveTo(center, props.lineWidth)
        ctx.lineTo(center, ControlBall.value.offsetHeight - props.lineWidth)
    } else {
        const center = ControlBall.value.offsetHeight / 2
        ctx.moveTo(props.lineWidth, center)
        ctx.lineTo(ControlBall.value.offsetWidth - props.lineWidth, center)
    }
    ctx.stroke()
}

// 2点間の距離算出
function calcDistance(x1, y1, x2, y2) {
    var distance = Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2))
    return distance
}

function touchstart(event) {
    event.preventDefault()
    if (data.value.isTouch === false) {
        var touch = event.targetTouches[0]
        var x1 = data.value.center.x
        var y1 = data.value.center.y
        const rect = ControlBall.value.getBoundingClientRect()
        var x2 = touch.clientX - rect.left
        var y2 = touch.clientY - rect.top
        if (props.direction === 'vertical') {
            y1 += data.value.modelPix
        } else {
            x1 += data.value.modelPix
        }
        if (calcDistance(x1, y1, x2, y2) < props.ballweight) {
            data.value.isTouch = true
            data.value.identifier = touch.identifier
            if (props.type == "variable") {
                model.value = 0
                data.value.model = 0
                data.value.modelPix = 0
            }
            draw()
        }
    }
}

function touchmove(event) {
    if (data.value.isTouch === true) {
        const rect = ControlBall.value.getBoundingClientRect()
        for (var i=0; i < event.changedTouches.length; i++) {
            var touch = event.changedTouches[i]
            if (touch.identifier === data.value.identifier) {
                if (props.direction === 'vertical') {
                    var y1 = data.value.center.y
                    var y2 = touch.clientY - rect.top
                    y2 = y2 < props.lineWidth ? props.lineWidth : y2 > ControlBall.value.offsetHeight - props.lineWidth ? ControlBall.value.offsetHeight - props.lineWidth : y2
                    if (y1 <= y2) {
                        const move = parseInt((y2 - y1) / data.value.center.y * -100)
                        model.value = move
                        data.value.model = move
                        data.value.modelPix = y2 - y1
                    } else {
                        const move = parseInt((y1 - y2) / data.value.center.y * 100)
                        model.value = move
                        data.value.model = move
                        data.value.modelPix = y2 - y1
                    }
                } else {
                    var x1 = data.value.center.x
                    var x2 = touch.clientX - rect.left
                    x2 = x2 < props.lineWidth ? props.lineWidth : x2 > ControlBall.value.offsetWidth - props.lineWidth ? ControlBall.value.offsetWidth - props.lineWidth : x2
                    if (x1 <= x2) {
                        const move = parseInt((x2 - x1) / data.value.center.x * 100)
                        model.value = move
                        data.value.model = move
                        data.value.modelPix = x2 - x1
                    } else {
                        const move = parseInt((x1 - x2) / data.value.center.x * -100)
                        model.value = move
                        data.value.model = move
                        data.value.modelPix = x2 - x1
                    }
                }
                draw()
                break
            }            
        }
    }
}

function touchend(event) {
    if (data.value.isTouch === true) {
        data.value.isTouch = false
        if (props.type == "variable") {
            model.value = 0
            data.value.model = 0
            data.value.modelPix = 0
        }
        draw()
    }
}

function mousedown(event) {
    console.log("mousedown")
}

function mousemove(event) {
    console.log("mousemove")
}

function mouseup(event) {
    console.log("mouseup")
}

</script>

<template>
    <canvas ref="ControlBall" class="ControlBall"
        @touchstart="touchstart" @touchmove="touchmove" @touchend="touchend"
        @mousedown="mousedown" @mousemove="mousemove" @mouseup="mouseup">
        {{ direction }}
    </canvas>
</template>