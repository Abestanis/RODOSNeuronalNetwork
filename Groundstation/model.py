import os
from argparse import ArgumentParser
from struct import calcsize, unpack

import numpy as np
import tensorflow as tf
from tensorflow import keras, lite


def saveModel(model, outputPath):
    basePath = os.path.splitext(outputPath)[0]
    model.save(basePath + '.h5')

    converter = lite.TFLiteConverter.from_keras_model(model)
    tfLiteModel = converter.convert()
    # Save the TF Lite model.
    with tf.io.gfile.GFile(basePath + '.tflite', 'wb') as f:
        f.write(tfLiteModel)


def main(dataFile, outputPath, epochs):
    inputSize = 3 * 4
    model = keras.Sequential([
        keras.layers.Input(shape=(inputSize,), name='sensor_input'),
        keras.layers.Dense(units=256, activation='relu'),
        keras.layers.Dense(units=192, activation='relu'),
        keras.layers.Dense(units=128, activation='relu'),
        keras.layers.Dense(units=3, activation='softmax')
    ])
    model.compile(optimizer='adam', loss='mean_squared_error')

    inputs = []
    outputs = []
    DATA_FORMAT = r'<IqB?ddddddddddddddd'
    DATA_LENGTH = calcsize(DATA_FORMAT)
    print('Loading training data...')
    with open(dataFile, 'rb') as dataFile:
        while True:
            data = dataFile.read(DATA_LENGTH)
            if len(data) != DATA_LENGTH:
                break
            dataPoints = unpack(DATA_FORMAT, data)
            if not dataPoints[3]:  # If not sensors ready
                continue
            inputs.append([
                dataPoints[7], dataPoints[8], dataPoints[9],  # Gyroscope
                dataPoints[4], dataPoints[5], dataPoints[6],  # Accelerometer
                dataPoints[15], dataPoints[13], dataPoints[14],  # Orientation
                dataPoints[18], dataPoints[16], dataPoints[17],  # Target orientation
            ])
            outputs.append([
                data[10], data[11], data[12]  # Servo output
            ])
    numDataPoints = len(inputs)
    print(f'Loaded {numDataPoints} data points.')

    inputs = np.array(inputs)
    outputs = np.array(outputs)
    mask = np.random.choice([False, True], numDataPoints, p=[0.9, 0.1])
    validationInputs = inputs[mask]
    inputs = inputs[~mask]
    validationOutputs = outputs[mask]
    outputs = outputs[~mask]
    model.fit(inputs, outputs, epochs=epochs, validation_data=(validationInputs, validationOutputs))
    saveModel(model, outputPath)


def qTraining(modelInputPath, modelOutputPath, controller):
    # Configuration parameters for the whole setup
    gamma = 0.99  # Discount factor for past rewards
    epsilon = 1.0  # Epsilon greedy parameter
    epsilonMin = 0.1  # Minimum epsilon greedy parameter
    epsilonMax = 1.0  # Maximum epsilon greedy parameter
    epsilonInterval = (
            epsilonMax - epsilonMin
    )  # Rate at which to reduce chance of random action being taken
    batchSize = 32  # Size of batch taken from replay buffer
    maxStepsPerEpisode = 1000  # Allow at max 50 Seconds per episode

    # The first model makes the predictions for Q-values which are used to
    # make a action.
    model = keras.models.load_model(modelInputPath)
    # Build a target model for the prediction of future rewards.
    # The weights of a target model get updated every 10000 steps thus when the
    # loss between the Q-values is calculated the target Q-value is stable.
    modelTarget = keras.models.load_model(modelInputPath)

    optimizer = keras.optimizers.Adam(learning_rate=0.00025, clipnorm=1.0)

    # Experience replay buffers
    stateHistory = []
    stateNextHistory = []
    rewardsHistory = []
    doneHistory = []
    episodeRewardHistory = []
    runningReward = 0
    episodeCount = 0
    tickCount = 0
    # Number of ticks to take random action and observe output
    epsilonRandomTicks = 50000
    # Number of ticks for exploration
    epsilonGreedyTicks = 1000000.0
    # Maximum replay length
    maxMemoryLength = 100000
    # Train the model after 4 actions
    updateAfterActions = 4
    # How often to update the target network
    updateTargetNetwork = 10000
    # Using huber loss for stability
    lossFunction = keras.losses.Huber()

    while True:  # Run until solved
        state = np.array(controller.resetTrainingState())
        episodeReward = 0

        for timeStep in range(1, maxStepsPerEpisode):
            tickCount += 1

            # Use epsilon-greedy for exploration
            if tickCount < epsilonRandomTicks or epsilon > np.random.rand(1)[0]:
                # Take random action
                output = np.random.uniform(low=-17, high=28, size=(3,))
            else:
                # Predict action Q-values
                # From environment state
                stateTensor = tf.convert_to_tensor(state)
                stateTensor = tf.expand_dims(stateTensor, 0)
                output = model(stateTensor, training=False)[0]

            # Decay probability of taking random action
            epsilon -= epsilonInterval / epsilonGreedyTicks
            epsilon = max(epsilon, epsilonMin)

            # Apply the sampled action in our environment
            stateNext, reward, done = controller.executeTrainingStep(output)
            stateNext = np.array(stateNext)

            episodeReward += reward

            # Save actions and states in replay buffer
            stateHistory.append(state)
            stateNextHistory.append(stateNext)
            doneHistory.append(done)
            rewardsHistory.append(reward)
            state = stateNext

            # Update every fourth tick and once batch size is over 32
            if tickCount % updateAfterActions == 0 and len(doneHistory) > batchSize:
                # Get indices of samples for replay buffers
                indices = np.random.choice(range(len(doneHistory)), size=batchSize)

                # Sample from replay buffer
                stateSample = np.array([stateHistory[i] for i in indices])
                stateNextSample = np.array([stateNextHistory[i] for i in indices])
                rewardsSample = [rewardsHistory[i] for i in indices]
                doneSample = tf.convert_to_tensor([float(doneHistory[i]) for i in indices])

                # Build the updated Q-values for the sampled future states
                # Use the target model for stability
                futureRewards = modelTarget.predict(stateNextSample)
                # Q value = reward + discount factor * expected future reward
                updatedQValues = rewardsSample + gamma * tf.reduce_max(futureRewards, axis=1)

                # If final tick set the last value to -1
                updatedQValues = updatedQValues * (1 - doneSample) - doneSample

                with tf.GradientTape() as tape:
                    # Train the model on the states and updated Q-values
                    qValues = model(stateSample)

                    # Apply the masks to the Q-values to get the Q-value for action taken
                    qAction = tf.reduce_sum(qValues, axis=1)
                    # Calculate loss between new Q-value and old Q-value
                    loss = lossFunction(updatedQValues, qAction)

                # Backpropagation
                grads = tape.gradient(loss, model.trainable_variables)
                optimizer.apply_gradients(zip(grads, model.trainable_variables))

            if tickCount % updateTargetNetwork == 0:
                # update the the target network with new weights
                modelTarget.set_weights(model.get_weights())
                # Log details
                print(f'Running reward: {runningReward:.2f} at episode {episodeCount}, '
                      f'tick count {tickCount}')

            # Limit the state and reward history
            if len(rewardsHistory) > maxMemoryLength:
                del rewardsHistory[:1]
                del stateHistory[:1]
                del stateNextHistory[:1]
                del doneHistory[:1]

            if done:
                break

        # Update running reward to check condition for solving
        episodeRewardHistory.append(episodeReward)
        if len(episodeRewardHistory) > 100:
            del episodeRewardHistory[:1]
        runningReward = np.mean(episodeRewardHistory)

        episodeCount += 1
        print(f'Episode {episodeCount}: Mean reward {runningReward:.2f}')
        if runningReward > 100:  # Condition to consider the task solved
            print(runningReward)
            print(f'Solved at episode {episodeCount}!')
            break
    saveModel(model, modelOutputPath)


if __name__ == '__main__':
    _parser = ArgumentParser(description='Creates and trains a new neuronal network.')
    _parser.add_argument('-e', '--epochs', help='The number of epochs to train for.', type=int)
    _parser.add_argument('-o', '--output', help='The path to store the trained model.',
                         default=os.path.join('models', 'model.tflite'))
    _parser.add_argument('-d', '--dataFile', required=True,
                         help='The file with the trainings data.')
    _args = _parser.parse_args()
    main(_args.dataFile, _args.output, _args.epochs)
